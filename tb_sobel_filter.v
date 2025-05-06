`timescale 1ns / 1ps

module tb_sobel_filter;

    // Parameters (match DUT)
    localparam BRAM_DATA_WIDTH = 32;
    localparam PIXEL_WIDTH     = 8;
    localparam ADDR_WIDTH      = 10;       // For 1024 words (64x64 pixels / 4 pixels/word)
    localparam IMG_WIDTH_BITS  = 16;
    localparam IMG_HEIGHT_BITS = 16;
    localparam KERNEL_SIZE     = 3;        // For verification logic if added

    // Test Image Dimensions
    localparam TEST_IMG_WIDTH  = 64;
    localparam TEST_IMG_HEIGHT = 64;
    localparam TEST_IMG_PIXELS = TEST_IMG_WIDTH * TEST_IMG_HEIGHT;
    // Calculate words needed, ensuring ceiling division for BRAM_DATA_WIDTH
    localparam PIXELS_PER_WORD = BRAM_DATA_WIDTH / PIXEL_WIDTH;
    localparam TEST_IMG_WORDS  = (TEST_IMG_PIXELS + PIXELS_PER_WORD - 1) / PIXELS_PER_WORD;


    // DUT Signals
    reg clk = 0;
    reg rst_n = 0;      // Active low reset
    reg start_tb = 0;   // Testbench controlled start signal
    reg [1:0] mode_tb = 2'b10; // Testbench controlled mode (e.g., |Gx|+|Gy|)
    reg [IMG_WIDTH_BITS-1:0] image_width_in_tb = TEST_IMG_WIDTH;
    reg [IMG_HEIGHT_BITS-1:0] image_height_in_tb = TEST_IMG_HEIGHT;
    wire busy_dut;      // DUT's busy signal
    wire done_dut;      // DUT's done signal

    // Input BRAM Interface Signals (DUT outputs these, TB responds)
    wire                  bram_in_ena_dut;
    wire [ADDR_WIDTH-1:0] bram_in_addr_dut;
    reg  [BRAM_DATA_WIDTH-1:0] bram_in_dout_tb; // Data provided by Testbench to DUT
    // wire                  bram_in_wea_dut;  // From DUT, should be 0
    // wire [BRAM_DATA_WIDTH-1:0] bram_in_dina_dut; // From DUT, should be 0

    // Output BRAM Interface Signals (DUT drives these, TB captures)
    wire                  bram_out_ena_dut;
    wire [ADDR_WIDTH-1:0] bram_out_addr_dut;
    wire [BRAM_DATA_WIDTH-1:0] bram_out_dina_dut;
    wire                  bram_out_wea_dut;

    // Simulate Input Memory (stores 32-bit words)
    reg [BRAM_DATA_WIDTH-1:0] input_bram_memory [0:TEST_IMG_WORDS-1];

    // Simulate Output Memory (stores 32-bit words)
    reg [BRAM_DATA_WIDTH-1:0] output_bram_memory [0:TEST_IMG_WORDS-1];

    // Instantiate Design Under Test (DUT)
    sobel_filter #(
        .BRAM_DATA_WIDTH(BRAM_DATA_WIDTH),
        .PIXEL_WIDTH(PIXEL_WIDTH),
        .ADDR_WIDTH(ADDR_WIDTH),
        .IMG_WIDTH_BITS(IMG_WIDTH_BITS),
        .IMG_HEIGHT_BITS(IMG_HEIGHT_BITS),
        .IMG_WIDTH_MAX(TEST_IMG_WIDTH),   // Max supported by line buffers in DUT
        .IMG_HEIGHT_MAX(TEST_IMG_HEIGHT)  // Max supported by line buffers in DUT
    ) dut_instance (
        .clk(clk),
        .rst_n(rst_n),
        .start(start_tb), // Connect TB's start to DUT's start
        .mode(mode_tb),   // Connect TB's mode to DUT's mode
        .image_width_in(image_width_in_tb),   // Connect TB's width to DUT's width input
        .image_height_in(image_height_in_tb), // Connect TB's height to DUT's height input
        .busy(busy_dut),
        .done(done_dut),

        .bram_in_ena(bram_in_ena_dut),
        .bram_in_addr(bram_in_addr_dut),
        .bram_in_dout(bram_in_dout_tb), // DUT reads data provided by this TB signal
        .bram_in_wea(/* bram_in_wea_dut */),   // DUT drives this, can be monitored
        .bram_in_dina(/* bram_in_dina_dut */), // DUT drives this, can be monitored

        .bram_out_ena(bram_out_ena_dut),
        .bram_out_addr(bram_out_addr_dut),
        .bram_out_dina(bram_out_dina_dut),
        .bram_out_wea(bram_out_wea_dut)
    );

    // Clock Generation (e.g., 50 MHz -> 20ns period)
    always #10 clk = ~clk; // Creates a 20ns period clock (50 MHz)

    // Input BRAM Read Simulation (Model with 1-cycle read latency)
    // When DUT asserts bram_in_ena_dut and provides bram_in_addr_dut,
    // this block will provide data on bram_in_dout_tb on the *next* clock cycle.
    reg bram_in_ena_dut_d1;         // Delayed enable signal
    reg [ADDR_WIDTH-1:0] bram_in_addr_dut_d1; // Delayed address signal

    always @(posedge clk) begin
         bram_in_ena_dut_d1 <= bram_in_ena_dut;
         bram_in_addr_dut_d1 <= bram_in_addr_dut;
         if (bram_in_ena_dut_d1) begin
             if (bram_in_addr_dut_d1 < TEST_IMG_WORDS) begin // Boundary check
                 bram_in_dout_tb <= input_bram_memory[bram_in_addr_dut_d1];
                 $display("TB: [%0t] Read from input_bram_memory[%d] = 0x%h", $time, bram_in_addr_dut_d1, input_bram_memory[bram_in_addr_dut_d1]);
             end else begin
                 bram_in_dout_tb <= {BRAM_DATA_WIDTH{1'bx}}; // Out of bounds read
                 $display("TB_ERROR: [%0t] Attempted read out of bounds from input_bram_memory. Addr: %d", $time, bram_in_addr_dut_d1);
             end
         end
    end

    // Output BRAM Write Simulation (Captures data written by DUT)
    always @(posedge clk) begin
        if (bram_out_ena_dut && bram_out_wea_dut) begin
            if (bram_out_addr_dut < TEST_IMG_WORDS) begin // Boundary check
                output_bram_memory[bram_out_addr_dut] <= bram_out_dina_dut;
                $display("TB: [%0t] DUT Writing 0x%h to output_bram_memory[%d]", $time, bram_out_dina_dut, bram_out_addr_dut);
            end else begin
                 $display("TB_ERROR: [%0t] DUT Attempted write out of bounds to output_bram_memory. Addr: %d, Data: %h", $time, bram_out_addr_dut, bram_out_dina_dut);
            end
        end
    end

    // Test Sequence
    integer r_idx, c_idx, word_idx, p_idx, byte_offset;
    reg [PIXEL_WIDTH-1:0] temp_pixel_data [0:TEST_IMG_PIXELS-1]; // Temporary storage for 8-bit pixels

    initial begin
        // 1. Initialize Testbench Input Memory
        $display("TB: Initializing testbench input memory with a gradient pattern...");
        // Create 8-bit pixel data
        for (r_idx = 0; r_idx < TEST_IMG_HEIGHT; r_idx = r_idx + 1) begin
             for (c_idx = 0; c_idx < TEST_IMG_WIDTH; c_idx = c_idx + 1) begin
                 temp_pixel_data[r_idx * TEST_IMG_WIDTH + c_idx] = (c_idx * 4) % 256; // Horizontal gradient
             end
        end

        // Pack 8-bit pixels into 32-bit BRAM words (assuming LSByte is pixel 0 of the word)
        for (word_idx = 0; word_idx < TEST_IMG_WORDS; word_idx = word_idx + 1) begin
            input_bram_memory[word_idx] = 0; // Initialize word
            for (byte_offset = 0; byte_offset < PIXELS_PER_WORD; byte_offset = byte_offset + 1) begin
                p_idx = word_idx * PIXELS_PER_WORD + byte_offset;
                if (p_idx < TEST_IMG_PIXELS) begin // Ensure we don't read past the pixel array
                    input_bram_memory[word_idx] = input_bram_memory[word_idx] | (temp_pixel_data[p_idx] << (byte_offset * PIXEL_WIDTH));
                end
            end
            // $display("TB: input_bram_memory[%d] = 0x%h", word_idx, input_bram_memory[word_idx]);
        end
        $display("TB: Testbench input memory initialized.");

        // 2. Apply Reset to DUT
        rst_n = 1'b1; // Start with reset de-asserted (often good practice)
        start_tb = 1'b0;
        mode_tb = 2'b10; // e.g., |Gx|+|Gy|
        image_width_in_tb = TEST_IMG_WIDTH;
        image_height_in_tb = TEST_IMG_HEIGHT;
        #5; // Wait a bit
        rst_n = 1'b0; // Assert active-low reset
        #50;          // Hold reset for a few clock cycles
        rst_n = 1'b1; // De-assert reset
        #50;          // Allow DUT to stabilize after reset

        // 3. Provide Start Pulse to DUT
        $display("TB: [%0t] Asserting start pulse for DUT...", $time);
        start_tb = 1'b1;
        #20; // Hold start pulse for one clock cycle (assuming 20ns clock period)
        start_tb = 1'b0;
        $display("TB: [%0t] Start pulse de-asserted.", $time);

        // 4. Wait for DUT to complete processing (or a timeout)
        // Monitor the 'done_dut' signal.
        // Add a timeout mechanism to prevent simulation from running indefinitely if 'done' never asserts.
        fork
            begin
                wait (done_dut == 1'b1);
                $display("TB: [%0t] DUT signaled 'done'.", $time);
            end
            begin
                # (TEST_IMG_PIXELS * 20 * 20); // Generous timeout (pixels * cycles_per_pixel_approx * clk_period)
                $display("TB_ERROR: [%0t] Simulation timed out waiting for 'done'.", $time);
                $finish; // End simulation on timeout
            end
        join_any // Proceed when either 'done' or timeout occurs

        // 5. (Optional) Verification of Output Memory
        // This would involve calculating the expected Sobel output for the input pattern
        // and comparing it with 'output_bram_memory'. This is complex for a full image.
        // For now, let's just print a few output words.
        $display("TB: First few words from output_bram_memory:");
        for (word_idx = 0; word_idx < 10 && word_idx < TEST_IMG_WORDS; word_idx = word_idx + 1) begin
            $display("TB: output_bram_memory[%d] = 0x%h", word_idx, output_bram_memory[word_idx]);
        end

        #100; // Allow some additional time for final signals to settle if needed
        $display("TB: Simulation finished.");
        $finish; // End the simulation
    end

endmodule
