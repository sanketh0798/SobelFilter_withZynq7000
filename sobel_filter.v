`timescale 1ns / 1ps

module sobel_filter #(
    // Parameters
    parameter BRAM_DATA_WIDTH = 32,       // BRAM data width (e.g., 32 bits)
    parameter PIXEL_WIDTH     = 8,        // Pixel data width (e.g., 8 bits for grayscale)
    parameter ADDR_WIDTH      = 10,       // Address width for BRAM (e.g., 10 for 1024 words)
    parameter IMG_WIDTH_BITS  = 16,       // Number of bits for image width input
    parameter IMG_HEIGHT_BITS = 16,       // Number of bits for image height input
    parameter IMG_WIDTH_MAX   = 64,       // Maximum image width supported by line buffers
    parameter IMG_HEIGHT_MAX  = 64        // Maximum image height supported
) (
    // Clock and Reset (Processing Domain)
    input wire clk,                       // Processing clock (e.g., 50 MHz)
    input wire rst_n,                     // Active Low Reset for processing logic

    // Control Interface (Direct Wires from GPIO / PL)
    input wire start,                     // Single cycle start pulse to begin processing
    input wire [1:0] mode,                // Sobel mode: 00: Gx, 01: Gy, 10: |Gx|+|Gy|
    input wire [IMG_WIDTH_BITS-1:0] image_width_in,  // Actual width of the image to process
    input wire [IMG_HEIGHT_BITS-1:0] image_height_in, // Actual height of the image to process
    output wire busy,                     // High when the filter is processing
    output wire done,                     // Pulsed high for one cycle when processing is complete

    // Input BRAM Interface (Port A - for reading source image)
    output wire                  bram_in_ena,       // Enable signal for input BRAM read
    output wire [ADDR_WIDTH-1:0] bram_in_addr,    // Word address for input BRAM read
    input wire [BRAM_DATA_WIDTH-1:0] bram_in_dout,  // Data read from input BRAM (32-bit word)
    output wire                  bram_in_wea,       // Write enable for input BRAM (tied low)
    output wire [BRAM_DATA_WIDTH-1:0] bram_in_dina,  // Data to write to input BRAM (unused)

    // Output BRAM Interface (Port A - for writing processed image)
    output wire                  bram_out_ena,      // Enable signal for output BRAM write
    output wire [ADDR_WIDTH-1:0] bram_out_addr,   // Word address for output BRAM write
    output wire [BRAM_DATA_WIDTH-1:0] bram_out_dina, // Data to write to output BRAM (32-bit word)
    output wire                  bram_out_wea       // Write enable for output BRAM
);

//--------------------------------------------------------------------------
// Internal Signals & Parameters
//--------------------------------------------------------------------------
localparam KERNEL_SIZE = 3;                                // Sobel kernel is 3x3
localparam BYTES_PER_WORD = BRAM_DATA_WIDTH / PIXEL_WIDTH; // Pixels per BRAM word (e.g., 32/8 = 4)
localparam CONV_WIDTH = 12;                               // Bit width for intermediate Gx/Gy results
localparam BYTE_LANE_WIDTH = 2;                           // Bits to select a byte within a word (log2(BYTES_PER_WORD))

// State Machine Definition (using localparams for Verilog-2001 compatibility)
localparam [2:0] IDLE           = 3'b000; // Waiting for start signal
localparam [2:0] READ_ROW_SETUP = 3'b001; // Initial state to fill line buffers for the first row
localparam [2:0] READ_PIXEL     = 3'b010; // State to initiate a BRAM read for the next word
localparam [2:0] PROCESS_WINDOW = 3'b011; // State to process the current pixel window
localparam [2:0] WRITE_PIXEL    = 3'b100; // State to write the convolved pixel to output BRAM
localparam [2:0] DONE_STATE     = 3'b101; // Processing finished

reg [2:0] current_state, next_state;    // State registers

// Latched Configuration (from input ports)
reg [IMG_WIDTH_BITS-1:0]  image_width;    // Latched image width
reg [IMG_HEIGHT_BITS-1:0] image_height;   // Latched image height

// Counters for pixel processing
reg [IMG_WIDTH_BITS-1:0]  pixel_col_cnt;  // Current column of the pixel being processed (0 to image_width-1)
reg [IMG_HEIGHT_BITS-1:0] pixel_row_cnt;  // Current row of the pixel being processed (0 to image_height-1)
reg [ADDR_WIDTH-1:0]      read_word_addr; // Calculated BRAM word address for reading
reg [BYTE_LANE_WIDTH-1:0] read_byte_lane; // Byte lane (0-3) of the current pixel within the read BRAM word

// Line Buffers (to store rows for 3x3 window) and Window Register
reg [PIXEL_WIDTH-1:0] line_buffer0 [0:IMG_WIDTH_MAX-1]; // Stores row (N-2)
reg [PIXEL_WIDTH-1:0] line_buffer1 [0:IMG_WIDTH_MAX-1]; // Stores row (N-1)
reg [PIXEL_WIDTH-1:0] line_buffer2 [0:IMG_WIDTH_MAX-1]; // Stores current row (N)
reg [PIXEL_WIDTH-1:0] window [0:KERNEL_SIZE-1][0:KERNEL_SIZE-1]; // 3x3 pixel window

// Pixel Extraction & BRAM Interface Registers
reg [BRAM_DATA_WIDTH-1:0] bram_read_data_reg;   // Latches data read from input BRAM
reg                       bram_read_data_valid; // Flag indicating valid data in bram_read_data_reg
reg [PIXEL_WIDTH-1:0]     current_pixel;        // Currently extracted 8-bit pixel

reg                       bram_in_ena_reg;      // Internal register for input BRAM enable
reg [ADDR_WIDTH-1:0]      bram_in_addr_reg;   // Internal register for input BRAM address
reg                       bram_out_ena_reg;     // Internal register for output BRAM enable
reg [ADDR_WIDTH-1:0]      bram_out_addr_reg;  // Internal register for output BRAM address
reg [BRAM_DATA_WIDTH-1:0] bram_out_dina_reg;  // Internal register for output BRAM data
reg                       bram_out_wea_reg;   // Internal register for output BRAM write enable

// Convolution Results
reg signed [CONV_WIDTH-1:0] Gx, Gy;             // Calculated Gx and Gy values (signed)
reg [PIXEL_WIDTH-1:0]       pixel_out;          // Final 8-bit convolved pixel output

// Loop variables (must be integer for Verilog-2001 loops)
integer i, j, c_offset;

// Read latency tracking: bram_in_ena_d1 is a delayed version of bram_in_ena_reg
reg bram_in_ena_d1;

// Temporary calculation variables (declared at module level for Verilog-2001 compatibility)
// For Convolution Calculation block
reg signed [CONV_WIDTH-1:0] calc_Gx_temp;
reg signed [CONV_WIDTH-1:0] calc_Gy_temp;
reg signed [CONV_WIDTH-1:0] calc_abs_gx_temp;
reg signed [CONV_WIDTH-1:0] calc_abs_gy_temp;
reg [CONV_WIDTH:0]          calc_sum_abs_temp; // Needs one extra bit for sum of abs

// For Main Sequential Logic block
reg [IMG_WIDTH_BITS-1:0] temp_buf_col_seq;
reg [IMG_HEIGHT_BITS-1:0] temp_output_row_seq;
reg [IMG_WIDTH_BITS-1:0] temp_output_col_seq;
reg [ADDR_WIDTH-1:0] temp_write_addr_seq;

//--------------------------------------------------------------------------
// Output Assignments (Connecting internal registers to module outputs)
//--------------------------------------------------------------------------
assign bram_in_ena  = bram_in_ena_reg;
assign bram_in_addr = bram_in_addr_reg;
assign bram_in_wea  = 1'b0;                             // Input BRAM is read-only from this module
assign bram_in_dina = {BRAM_DATA_WIDTH{1'b0}};         // No data written to input BRAM

assign bram_out_ena  = bram_out_ena_reg;
assign bram_out_addr = bram_out_addr_reg;
assign bram_out_dina = bram_out_dina_reg;
assign bram_out_wea  = bram_out_wea_reg;

assign busy = (current_state != IDLE && current_state != DONE_STATE); // Busy if not idle or done
assign done = (current_state == DONE_STATE);                          // Done when in DONE_STATE

//--------------------------------------------------------------------------
// Read Latency & Data Valid Handling (Sequential Logic)
// Assumes 1 cycle read latency from BRAM after enable.
//--------------------------------------------------------------------------
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        bram_in_ena_d1 <= 1'b0;
        bram_read_data_valid <= 1'b0;
        bram_read_data_reg <= {BRAM_DATA_WIDTH{1'b0}};
    end else begin
        bram_in_ena_d1 <= bram_in_ena_reg; // Delay the enable signal by one cycle
        if (bram_in_ena_d1) begin          // If enable was asserted in the previous cycle...
            bram_read_data_valid <= 1'b1;  // ...then data is valid in the current cycle.
            bram_read_data_reg <= bram_in_dout; // Latch the incoming data.
        end else begin
            bram_read_data_valid <= 1'b0;  // Otherwise, no new valid data.
        end
    end
end

//--------------------------------------------------------------------------
// State Machine (Sequential Part - State Register Update)
//--------------------------------------------------------------------------
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        current_state <= IDLE;
    end else begin
        current_state <= next_state; // Update current state based on combinational logic
    end
end

//--------------------------------------------------------------------------
// State Machine (Combinational Part - Next State Logic & Control Signal Generation)
//--------------------------------------------------------------------------
// Using @(*) for automatic sensitivity list. If issues arise with older tools,
// an explicit sensitivity list would be needed here.
always @(*) begin
    // Default assignments for control signals (active low/off by default)
    next_state = current_state;        // Default: remain in current state
    bram_in_ena_reg = 1'b0;            // Default: input BRAM read disabled
    bram_out_ena_reg = 1'b0;           // Default: output BRAM access disabled
    bram_out_wea_reg = 1'b0;           // Default: output BRAM write disabled

    case (current_state)
        IDLE: begin
            if (start) begin            // If start pulse is received
                next_state = READ_ROW_SETUP; // Transition to initial row setup
            end
        end

        READ_ROW_SETUP: begin
             bram_in_ena_reg = 1'b1;     // Enable input BRAM to read initial data
             // Wait until enough data is read and processed to form the first 3x3 window
             if (pixel_row_cnt == 2 && pixel_col_cnt == 2 && bram_read_data_valid) begin
                 next_state = PROCESS_WINDOW; // Ready to process first full window
             // If data just arrived for a previous read, or if no read was pending,
             // and we are not yet ready for full processing, go to read next pixel/word.
             end else if (bram_read_data_valid || !bram_in_ena_d1) begin
                  next_state = READ_PIXEL;
             end
             // else: Stay in READ_ROW_SETUP, waiting for BRAM data if a read is active
        end

        READ_PIXEL: begin
            bram_in_ena_reg = 1'b1;     // Enable input BRAM to read the next data word
            next_state = PROCESS_WINDOW; // Transition to process after data arrives
        end

        PROCESS_WINDOW: begin
            if (bram_read_data_valid) begin // If new data has arrived from BRAM
                // Pixel extraction and line buffer updates happen in the main sequential block
                // Check if the current window corresponds to a non-border pixel
                 if (pixel_row_cnt > 0 && pixel_col_cnt > 0) begin // Check if not first row/col for output
                    // A convolved pixel_out is calculated combinationally based on the window
                    next_state = WRITE_PIXEL; // Proceed to write the calculated pixel
                 end else begin // Current window is for a border input pixel, no output generated yet
                     // Determine if we need to read more data or if the image is done
                     if (pixel_col_cnt == image_width - 1) begin // If at the end of the current row
                         if (pixel_row_cnt == image_height - 1) begin // If also at the end of the image
                              next_state = DONE_STATE; // All border pixels processed, image complete
                         end else begin // End of row, but not end of image
                              next_state = READ_PIXEL; // Read data for the start of the next row
                         end
                     end else begin // Not at the end of the current row
                         // Check if the next pixel is in a new BRAM word
                         if ((pixel_col_cnt % BYTES_PER_WORD) == (BYTES_PER_WORD - 1)) begin
                             next_state = READ_PIXEL; // Need to read a new word
                         end else begin
                             next_state = PROCESS_WINDOW; // Next pixel is in the same word, wait for next clock edge
                                                          // to update counters and process it sequentially.
                         end
                     end
                 end
            end else begin // If BRAM data is not yet valid for the current read request
                 next_state = PROCESS_WINDOW; // Stay in this state, waiting for data
            end
        end

        WRITE_PIXEL: begin
             // Output BRAM address and data are prepared in the sequential block
             // when transitioning *into* this state.
             bram_out_ena_reg = 1'b1; // Enable output BRAM for access
             bram_out_wea_reg = 1'b1; // Assert write enable for output BRAM

             // After the write cycle, determine the next action
             if (pixel_col_cnt == image_width - 1) begin // If at the end of the current output row
                 if (pixel_row_cnt == image_height - 1) begin // If also at the end of the image
                      next_state = DONE_STATE; // All pixels processed and written
                 end else begin // End of row, but not end of image
                      next_state = READ_PIXEL; // Read data for the start of the next row
                 end
             end else begin // Not at the end of the current output row
                  // Check if the next input pixel (which forms the next output) requires a new BRAM read
                 if ((pixel_col_cnt % BYTES_PER_WORD) == (BYTES_PER_WORD - 1)) begin
                     next_state = READ_PIXEL;
                 end else begin
                     next_state = PROCESS_WINDOW; // Next pixel is in the same word
                 end
             end
        end

        DONE_STATE: begin
            next_state = IDLE;             // Return to idle state
        end
        default: next_state = IDLE;        // Default case for safety
    endcase
end

//--------------------------------------------------------------------------
// Convolution Calculation (Combinational Logic)
// This block calculates Gx, Gy, and the final pixel_out based on the current 'window'
//--------------------------------------------------------------------------
// 'p' is a wire array to hold sign-extended window pixels for calculation
wire signed [PIXEL_WIDTH:0] p [0:KERNEL_SIZE-1][0:KERNEL_SIZE-1];

// Generate block to perform sign-extension of window pixels
generate
    genvar i_gen, j_gen; // genvar is Verilog-2001 for generate loops
    for (i_gen = 0; i_gen < KERNEL_SIZE; i_gen = i_gen + 1) begin
        for (j_gen = 0; j_gen < KERNEL_SIZE; j_gen = j_gen + 1) begin
            // window[i][j] is 8-bit unsigned. p[i][j] becomes 9-bit signed.
            assign p[i_gen][j_gen] = {1'b0, window[i_gen][j_gen]};
        end
    end
endgenerate

// Combinational logic for Sobel Gx, Gy, and magnitude calculation
always @(*) begin // Sensitivity list includes 'p' (implicitly all its elements) and 'mode'
    // Use module-level temporary registers for calculations within this block
    // Sobel Gx kernel: [-1 0 +1; -2 0 +2; -1 0 +1]
    calc_Gx_temp = (p[0][2] + (p[1][2]<<1) + p[2][2]) - (p[0][0] + (p[1][0]<<1) + p[2][0]);

    // Sobel Gy kernel: [+1 +2 +1; 0 0 0; -1 -2 -1]
    calc_Gy_temp = (p[0][0] + (p[0][1]<<1) + p[0][2]) - (p[2][0] + (p[2][1]<<1) + p[2][2]);

    // Assign to module-level Gx, Gy registers (useful for debugging/ILA viewing)
    Gx = calc_Gx_temp;
    Gy = calc_Gy_temp;

    // Calculate absolute values of Gx and Gy
    calc_abs_gx_temp = (calc_Gx_temp < 0) ? -calc_Gx_temp : calc_Gx_temp;
    calc_abs_gy_temp = (calc_Gy_temp < 0) ? -calc_Gy_temp : calc_Gy_temp;

    // Combine Gx and Gy based on mode
    case (mode)
        2'b00: calc_sum_abs_temp = {1'b0, calc_abs_gx_temp}; // Gx only (magnitude)
        2'b01: calc_sum_abs_temp = {1'b0, calc_abs_gy_temp}; // Gy only (magnitude)
        2'b10: calc_sum_abs_temp = calc_abs_gx_temp + calc_abs_gy_temp; // |Gx| + |Gy| approximation
        default: calc_sum_abs_temp = calc_abs_gx_temp + calc_abs_gy_temp; // Default to combined
    endcase

    // Saturate the result to PIXEL_WIDTH (8 bits)
    if (calc_sum_abs_temp >= (1 << PIXEL_WIDTH)) begin // If sum_abs >= 256
         pixel_out = {PIXEL_WIDTH{1'b1}};           // Saturate to max value (255)
    end else begin
         pixel_out = calc_sum_abs_temp[PIXEL_WIDTH-1:0]; // Truncate to PIXEL_WIDTH
    end
end


//--------------------------------------------------------------------------
// Main Sequential Logic Block
// Handles: Counter updates, Latching configuration, Pixel Extraction,
//          Line Buffer shifts, Window register updates, Output BRAM data preparation.
//--------------------------------------------------------------------------
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        // Reset all relevant state registers and counters
        pixel_col_cnt <= {IMG_WIDTH_BITS{1'b0}};
        pixel_row_cnt <= {IMG_HEIGHT_BITS{1'b0}};
        image_width <= {IMG_WIDTH_BITS{1'b0}};
        image_height <= {IMG_HEIGHT_BITS{1'b0}};
        bram_in_addr_reg <= {ADDR_WIDTH{1'b0}};
        bram_out_addr_reg <= {ADDR_WIDTH{1'b0}};
        bram_out_dina_reg <= {BRAM_DATA_WIDTH{1'b0}};
        current_pixel <= {PIXEL_WIDTH{1'b0}};
        read_word_addr <= {ADDR_WIDTH{1'b0}};      // Explicitly reset
        read_byte_lane <= {BYTE_LANE_WIDTH{1'b0}};// Explicitly reset

        // Reset line buffers
        for (i = 0; i < IMG_WIDTH_MAX; i=i+1) begin
            line_buffer0[i] <= {PIXEL_WIDTH{1'b0}};
            line_buffer1[i] <= {PIXEL_WIDTH{1'b0}};
            line_buffer2[i] <= {PIXEL_WIDTH{1'b0}};
        end
        // Reset window register
        for (i=0; i<KERNEL_SIZE; i=i+1) begin
            for (j=0; j<KERNEL_SIZE; j=j+1) begin
                window[i][j] <= {PIXEL_WIDTH{1'b0}};
            end
        end
    end else begin // Normal clocked operation

        // Latch configuration and reset counters at the beginning of processing
        if (current_state == IDLE && next_state == READ_ROW_SETUP) begin
             image_width  <= image_width_in;
             image_height <= image_height_in;
             // Reset pixel counters for the new image
             pixel_col_cnt <= {IMG_WIDTH_BITS{1'b0}};
             pixel_row_cnt <= {IMG_HEIGHT_BITS{1'b0}};
             // Initialize BRAM addresses
             bram_in_addr_reg <= {ADDR_WIDTH{1'b0}}; // Start reading input BRAM from address 0
             bram_out_addr_reg <= {ADDR_WIDTH{1'b0}};// Output BRAM address also starts at 0 (or adjusted for borders)
        end

        // Calculate and set the input BRAM read address when a read is initiated
        if (next_state == READ_PIXEL || next_state == READ_ROW_SETUP) begin
             // Address is based on the *current* pixel counters, as they point to the
             // next pixel that needs to be fetched.
             read_word_addr = (pixel_row_cnt * image_width + pixel_col_cnt) / BYTES_PER_WORD;
             bram_in_addr_reg <= read_word_addr; // Set the BRAM address for the read
        end

        // When in PROCESS_WINDOW state and new BRAM data is valid:
        if (current_state == PROCESS_WINDOW && bram_read_data_valid) begin
             // Determine which byte within the 32-bit word corresponds to the current pixel
             read_byte_lane = (pixel_row_cnt * image_width + pixel_col_cnt) % BYTES_PER_WORD;

             // Extract the 8-bit pixel from the 32-bit BRAM data word
             case (read_byte_lane)
                 2'b00: current_pixel <= bram_read_data_reg[7:0];    // Byte 0
                 2'b01: current_pixel <= bram_read_data_reg[15:8];   // Byte 1
                 2'b10: current_pixel <= bram_read_data_reg[23:16];  // Byte 2
                 2'b11: current_pixel <= bram_read_data_reg[31:24];  // Byte 3
                 default: current_pixel <= {PIXEL_WIDTH{1'b0}};      // Should not happen
             endcase

             // Shift Line Buffers:
             // Data moves from line_buffer2 -> line_buffer1 -> line_buffer0
             // Must be done element by element for synthesis.
             for (i = 0; i < IMG_WIDTH_MAX; i = i + 1) begin
                 line_buffer0[i] <= line_buffer1[i];
                 line_buffer1[i] <= line_buffer2[i];
             end
             // Store the newly extracted 'current_pixel' into the current column of line_buffer2
             line_buffer2[pixel_col_cnt] <= current_pixel;

             // Update the 3x3 'window' register if enough rows/columns have been processed
             if (pixel_row_cnt >= KERNEL_SIZE - 1 && pixel_col_cnt >= KERNEL_SIZE - 1) begin
                 // The window is formed from the most recent KERNEL_SIZE columns
                 // of line_buffer0, line_buffer1, and line_buffer2.
                 for (c_offset = 0; c_offset < KERNEL_SIZE; c_offset = c_offset + 1) begin
                     // temp_buf_col_seq calculates the column index in the line buffers
                     // corresponding to the window columns.
                     temp_buf_col_seq = pixel_col_cnt - (KERNEL_SIZE - 1) + c_offset;
                     window[0][c_offset] <= line_buffer0[temp_buf_col_seq]; // Top row of window
                     window[1][c_offset] <= line_buffer1[temp_buf_col_seq]; // Middle row of window
                     window[2][c_offset] <= line_buffer2[temp_buf_col_seq]; // Bottom row of window
                 end
             end

             // Increment pixel counters *after* the current pixel has been processed
             if (pixel_col_cnt == image_width - 1) begin // If at the end of a column
                 pixel_col_cnt <= {IMG_WIDTH_BITS{1'b0}}; // Reset column counter
                 if (pixel_row_cnt < image_height - 1) begin // If not the last row
                     pixel_row_cnt <= pixel_row_cnt + 1;    // Increment row counter
                 end
             end else begin // Not at the end of a column
                 pixel_col_cnt <= pixel_col_cnt + 1;        // Increment column counter
             end
        end // end if(current_state == PROCESS_WINDOW && bram_read_data_valid)

        // When preparing to write a pixel (transitioning into WRITE_PIXEL state):
        if (next_state == WRITE_PIXEL) begin
             // The output pixel corresponds to the center of the window that was just processed.
             // The pixel_row_cnt and pixel_col_cnt have already been incremented to point
             // to the *next* input pixel. So, we need to calculate the output address
             // based on the pixel that *was* the center of the last window.
             if (pixel_col_cnt == 0) begin // If counters just wrapped to start of a new input line
                 temp_output_row_seq = pixel_row_cnt - 1; // Output was for the previous input line
                 temp_output_col_seq = image_width - 1;   // Output was for the last column of that line
             end else begin // Input counters are mid-line
                 temp_output_row_seq = pixel_row_cnt;     // Output was for the current input line
                 temp_output_col_seq = pixel_col_cnt - 1; // Output was for the previous input column
             end

             // Calculate the word address in output BRAM for this output pixel
             temp_write_addr_seq = (temp_output_row_seq * image_width + temp_output_col_seq) / BYTES_PER_WORD;
             bram_out_addr_reg <= temp_write_addr_seq; // Set the output BRAM address

             // Prepare the 32-bit data word for output BRAM.
             // **Simplified Write:** Places the 8-bit 'pixel_out' into the lowest byte.
             // This overwrites other bytes in the 32-bit word. A full implementation
             // would require read-modify-write or byte enables if supported.
             // 'pixel_out' is driven by the combinational convolution block.
             bram_out_dina_reg <= {{(BRAM_DATA_WIDTH-PIXEL_WIDTH){1'b0}}, pixel_out};
         end

    end // end else (!rst_n) normal operation
end // end always@

endmodule
