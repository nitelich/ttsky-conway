/* ************************************************************************* *
 * @file   peripheral_library.v
 * @author nitelich
 * @since  2024-06-10
 * @brief
 *
 * A collection of MCU-adjacent peripherals using only "low level" primitive
 * functions such as bitwise logic, multiplexers (as inferred from ternary
 * expressions), and other simple combinatorial and register logic.
 *
 * @details
 *
 * Contains implementations of the following high-level modules:
 *   - Active Bit Count
 *   - Shift Register
 *      - Linear Feedback Shift Register
 *      - Alternating Step Generator
 *   - VGA Controller Peripheral
 *   - Conway's Game of Life Grid
 *   - Tiny Tapeout Conway's Game of Life
 *
 * These high-level modules require a number of helper low-level modules:
 *   - Conway's Game of Life Cell
 *
 * ************************************************************************* */

`default_nettype none

/* Helper Macros *********************************************************** */

/** Bit width of neighbor sets for Conway cells.*/
`define CONWAY_WIDTH       8
/** Count width of neighbor sets for Conway cells.*/
`define CONWAY_COUNT       4
/** Conway's Game of Life standard birth rule, 0x008. */
`define CONWAY_BORN        9'h008
/** Conway's Game of Life standard survival rule, 0x00C.*/
`define CONWAY_SURVIVE     9'h00C

/** VGA positional data width, holding a maximum value of 800.*/
`define VGA_WIDTH          10

/**
 * Wrapping-safe cell lookup from X and Y coordinates.
 * @param[in] i The X coordinate.
 * @param[in] j The Y coordinate.
 * @return The cell id.
 */
`define C_ID(i, j) ((((j + HEIGHT) % HEIGHT) * WIDTH) + ((i + WIDTH) % WIDTH))

/**
 * For power-of-two pairwise combination circuits (CLZ, Active Bits),
 * the most significant bit for a specific generation and pair's sum.
 * @param[in] g The power of 2 generation.
 * @param[in] p The pair index.
 * @return The appropriate bit index value.
 */
`define SUM_MSB(g, p) ((((p) + 1) * ((g) + 2)) - 1)
/**
 * For power-of-two pairwise combination circuits (CLZ, Active Bits),
 * the least significant bit for a specific generation and pair's sum.
 * @param[in] g The power of 2 generation.
 * @param[in] p The pair index.
 * @return The appropriate bit index value.
 */
`define SUM_LSB(g, p) ((p) * ((g) + 2))
/**
 * For power-of-two pairwise combination circuits (CLZ, Active Bits),
 * the most significant bit for a specific generation and pair's left item.
 * @param[in] g The power of 2 generation.
 * @param[in] p The pair index.
 * @return The appropriate bit index value.
 */
`define LEFT_MSB(g, p) ((((2 * (p)) + 2) * ((g) + 1)) - 1)
/**
 * For power-of-two pairwise combination circuits (CLZ, Active Bits),
 * the least significant bit for a specific generation and pair's left item.
 * @param[in] g The power of 2 generation.
 * @param[in] p The pair index.
 * @return The appropriate bit index value.
 */
`define LEFT_LSB(g, p) (((2 * (p)) + 1) * ((g) + 1))
/**
 * For power-of-two pairwise combination circuits (CLZ, Active Bits),
 * the most significant bit for a specific generation and pair's right item.
 * @param[in] g The power of 2 generation.
 * @param[in] p The pair index.
 * @return The appropriate bit index value.
 */
`define RIGHT_MSB(g, p) ((((2 * (p)) + 1) * ((g) + 1)) - 1)
/**
 * For power-of-two pairwise combination circuits (CLZ, Active Bits),
 * the least significant bit for a specific generation and pair's right item.
 * @param[in] g The power of 2 generation.
 * @param[in] p The pair index.
 * @return The appropriate bit index value.
 */
`define RIGHT_LSB(g, p) ((2 * (p)) * ((g) + 1))

/* Primary Modules ********************************************************* */

/**
 * Tiny Tapeout implementation of a VGA-display Conway's Game of Life.
 *
 * This module is interactive, allowing for input-pin based selector
 * movement around the grid, setting and clearing of individual cells,
 * and randomization of the full field.
 *
 * Dedicated inputs are used for meta control - colors, free-run control,
 * and single step control. Dedicated outputs are used for VGA.
 * Bi-directional pins are all utilized as inputs for interaction.
 *
 * @param[in] ui_in Dedicated inputs. Utilized as follows:
 *   - ui[0] = FG Advance
 *     On the rising edge of this signal, advances the foreground color in
 *     the color wheel. The selector color is a static offset from the
 *     foreground color, skipping over the background color. No protection
 *     exists for the foreground and background color being equal, though.
 *   - ui[1] = BG Advance
 *     On the rising edge of this signal, advances the background color in
 *     the color wheel.
 *   - ui[5:2] = Free-run Divisor
 *     The Conway grid is, by default, updated once per vertical sync, giving
 *     60Hz update. This allows for any divisor between /1 and /16, allowing
 *     for a minimum 3.75Hz update. If slower update is desired, use the
 *     external hold and step method.
 *   - ui[6] = External Hold
 *     When asserted, the Conway cells are only updated on an external step.
 *   - ui[7] = External Step
 *     When an external hold is active, acts as a single step for game state.
 * @param[out] uo_out Dedicated outputs. Utilized as follows:
 *   - uo[0] = R1
 *     Red output for Tiny VGA, bit 1
 *   - uo[1] = G1
 *     Green output for Tiny VGA, bit 1
 *   - uo[2] = B1
 *     Blue output for Tiny VGA, bit 1
 *   - uo[3] = Vertical Sync
 *     Vertical sync signal for Tiny VGA
 *   - uo[4] = R0
 *     Red output for Tiny VGA, bit 0
 *   - uo[5] = G0
 *     Green output for Tiny VGA, bit 0
 *   - uo[6] = B0
 *     Blue output for Tiny VGA, bit 0
 *   - uo[7] = Horizontal Sync
 *     Horizontal sync signal for Tiny VGA
 * @param[in] uio_in Bidirectional pins when in input mode. See uio_oe for
 * implementation details of the bidirectional pins.
 * @param[out] uio_out Bidirectional pins when in output mode. See uio_oe for
 * implementation details of the bidirectional pins.
 * @param[out] uio_oe Bidirectional pins' direction (0 = input, 1 = output).
 * Bidirectional pins are utilized as follows:
 *   - uio[0] = Selection up (always input)
 *     On a rising edge, moves the selection up one cell, wrapping when appropriate.
 *   - uio[1] = Selection down (always input)
 *     On a rising edge, moves the selection down one cell, wrapping when appropriate.
 *   - uio[2] = Selection left (always input)
 *     On a rising edge, moves the selection left one cell, wrapping when appropriate.
 *   - uio[3] = Selection right (always input)
 *     On a rising edge, moves the selection right one cell, wrapping when appropriate.
 *   - uio[4] = Target state (always input)
 *     When updating the selected cell, the target cell state.
 *   - uio[5] = Update target (always input)
 *     On a rising edge, updates the state of the selected cell to the target state.
 *     Takes precedence over randomization.
 *   - uio[6] = Randomize all (always input)
 *     On a rising edge, triggers a randomization of all cell contents.
 *   - uio[7] = Selection to origin (always input)
 *     On a rising edge, moves the selection to cell (0, 0).
 * @param[in] ena Enable pin for this module (over other modules in the Tiny
 * Tapeout silicon). Note that, in normal operation, this value is 1 any time
 * this module is powered.
 * @param[in] clk Master clock. While any value between 3Hz and 50MHz may be
 * generated and used for this signal (a theoretical max clock rate for the
 * CPU core is unknown as of this writing), a value of 9.216MHz should be
 * used to allow for proper clocking of the SPI peripheral at 9.216MHz and
 * corresponding CPU speed of 256kHz. Other speeds may be used, bounded by
 * a 12MHz maximum SPI speed (12MHz master clock), or utilization of UART
 * and system tick peripherals. UART operation relies upon being able to
 * achieve well-defined baud rates within some percentage of precision, and
 * the 256kHz CPU clock is defined because 128kHz is a well-defined baud rate.
 * System tick operation assumes a 256kHz CPU clock to simplify timer design,
 * and will run fast or slow based on adjusting the CPU clock.
 * @param[in] rst_n Active low master reset.
 * @param[out] vga_blank As an extension, an indication of when VGA output is blanked.
 * @note 2025-07-02 Tested in full via MiSTer core targeting this module.
 */
module tt_um_nitelich_conway  (input  wire [7:0] ui_in,   output wire [7:0] uo_out,
      input wire [7:0] uio_in, output wire [7:0] uio_out, output wire [7:0] uio_oe,
      input wire ena,          input  wire       clk,     input  wire       rst_n
      );

   // ----- Constants -----
   localparam FIELD_DEPTH   = 3;                   /**< Master depth selection for GoL Tiny Tapeout field.*/
   localparam FIELD_DEPTH_2 = 2 * FIELD_DEPTH;     /**< Corresponding linear length of GoL Tiny Tapeout field.*/
   localparam FIELD_LENGTH  = 2 ** FIELD_DEPTH_2;  /**< Linear length addressing size for GoL Tiny Tapeout field.*/
   localparam FIELD_COLOR   = 4;                   /**< The depth of VGA colors allowed for the GoL Tiny Tapeout field.*/
   localparam FIELD_C_LEN   = 2 ** FIELD_COLOR;    /**< Corresponding total count of VGA colors for GoL Tiny Tapeout field.*/
   /**
    * The power-of-two magnitude of size for pixels on the VGA field.
    * This is floor(log2(min(width, height) / cells)).
    * ceil(x) - 1 ~= floor(x) for all reasonable choices of x here.
    */
   localparam FIELD_PIX_POW2= $clog2(480 / (2 ** FIELD_DEPTH)) - 1;
   /**
    * The number of pixels used by the GoL playing field.
    * For a (2**3)-cell-wide playing field, this is 256 (2**(3 + 5)).
    */
   localparam [`VGA_WIDTH - 1:0] FIELD_PIX_LEN = 10'd2 ** (FIELD_PIX_POW2 + FIELD_DEPTH);
   localparam [`VGA_WIDTH - 1:0] FIELD_PIX_DX  = (10'd640 - FIELD_PIX_LEN) / 10'd2;  /**< Sized VGA X centering offset.*/
   localparam [`VGA_WIDTH - 1:0] FIELD_PIX_DY  = (10'd480 - FIELD_PIX_LEN) / 10'd2;  /**< Sized VGA Y centering offset.*/
   
   // Note: Colors are based on 4-bit "web colors" list, as listed below.
   //       White, Silver, Grey, Black, Red, Maroon, Yellow, Olive, Lime,
   //       Green, Aqua, Teal, Blue, Navy, Fuchsia, Purple
   
   /** Color look-up table for red component of pixel data, high bit.*/
   wire [FIELD_C_LEN - 1:0] COLOR_LUT_RED_HI   = 16'b1100000011110011;
   /** Color look-up table for red component of pixel data, low bit.*/
   wire [FIELD_C_LEN - 1:0] COLOR_LUT_RED_LO   = 16'b0100000001010101;
   /** Color look-up table for green component of pixel data, high bit.*/
   wire [FIELD_C_LEN - 1:0] COLOR_LUT_GREEN_HI = 16'b0000111111000011;
   /** Color look-up table for green component of pixel data, low bit.*/
   wire [FIELD_C_LEN - 1:0] COLOR_LUT_GREEN_LO = 16'b0000010101000101;
   /** Color look-up table for blue component of pixel data, high bit.*/
   wire [FIELD_C_LEN - 1:0] COLOR_LUT_BLUE_HI  = 16'b1111110000000011;
   /** Color look-up table for blue component of pixel data, low bit.*/
   wire [FIELD_C_LEN - 1:0] COLOR_LUT_BLUE_LO  = 16'b0101010000000101;

   // ----- Local Variables -----
   genvar i;            /**< Loop integer.*/
   wire clk_mask;       /**< Masked clock for updating the field.*/
   reg  clk_next;       /**< Vertical sync based clock tick.*/
   wire rst_master;     /**< Master reset (combination of reset and enable.*/
   reg  rst_or_update;  /**< Indication to reset or update the field value.*/
   wire vga_x_ok;       /**< VGA is in the horizontal field print region.*/
   wire vga_y_ok;       /**< VGA is in the vertical field print region.*/
   wire cell_draw_orig; /**< State of VGA print cell.*/
   wire box_sel;        /**< Whether in the "selection box" portion of printing a cell.*/
   reg  step_val;       /**< Filter for rising edge of single step.*/
   reg  step_do;        /**< Indication to perform a single step.*/
   reg  advance_fg;     /**< Filter for rising edge of FG color advance.*/
   reg  advance_bg;     /**< Filter for rising edge of BG color advance.*/
   reg  update_val;     /**< Filter for rising edge of cell state update.*/
   reg  update_random;  /**< Indicates whether cell update is set or random.*/
   reg  update_select;  /**< Filter for rising edge of any selection update.*/
   wire v_sync;         /**< Vertical sync signal from VGA module.*/
   reg  v_sync_update;  /**< Rising edge detection of vertical sync.*/
   reg  [3:0]                 count_sync;    /**< Count of vertical syncs.*/
   wire [7:0]                 psrng_data;    /**< Current data output from the PSRNG.*/
   reg  [FIELD_DEPTH - 1:0]   select_x;      /**< Current user-selected cell X index.*/
   reg  [FIELD_DEPTH - 1:0]   select_y;      /**< Current user-selected cell Y index.*/
   reg  [FIELD_COLOR - 1:0]   color_fg;      /**< User-set FG color index.*/
   reg  [FIELD_COLOR - 1:0]   color_bg;      /**< User-set BG color index.*/
   wire [FIELD_COLOR - 1:0]   color_sel;     /**< Selection color index.*/
   wire [FIELD_COLOR - 1:0]   color_index;   /**< Color index of VGA print cell.*/
   wire [FIELD_DEPTH_2 - 1:0] vga_linear;    /**< Linear address of VGA print cell.*/
   wire [FIELD_DEPTH_2 - 1:0] select_linear; /**< Linear address of selected cell.*/
   reg  [FIELD_DEPTH_2 - 1:0] count_psrng;   /**< Counter for randomizing the field.*/
   wire [`VGA_WIDTH - 1:0]    vga_x;         /**< Current VGA X pixel.*/
   wire [`VGA_WIDTH - 1:0]    vga_x_off;     /**< VGA X pixel, offset to center the field.*/
   wire [`VGA_WIDTH - 1:0]    vga_y;         /**< Current VGA Y pixel.*/
   wire [`VGA_WIDTH - 1:0]    vga_y_off;     /**< VGA Y pixel, offset to center the field.*/
   reg  [FIELD_LENGTH - 1:0]  set_update;    /**< Force update state for the field.*/
   wire [FIELD_LENGTH - 1:0]  set_current;   /**< Last seen state for the field.*/

   // ----- Combinatorial Logic -----
   
   // All UIO pin directions are static inputs.
   // Also assign unused UIO outputs to 0.
   assign uio_oe  = 8'd0;
   assign uio_out = 8'd0;
   
   // Set general purpose outputs
   assign uo_out[0] = COLOR_LUT_RED_HI  [color_index];
   assign uo_out[1] = COLOR_LUT_GREEN_HI[color_index];
   assign uo_out[2] = COLOR_LUT_BLUE_HI [color_index];
   assign uo_out[4] = COLOR_LUT_RED_LO  [color_index];
   assign uo_out[5] = COLOR_LUT_GREEN_LO[color_index];
   assign uo_out[6] = COLOR_LUT_BLUE_LO [color_index];
   assign uo_out[7] = v_sync;
   
   // Determine master reset
   assign rst_master = ~rst_n | ~ena;
   
   // Determine masked clock for single step mode
   // Update either on vertical sync based clock counter or
   // the need for immediate updates.
   // Mask the vertical sync based clock with the single step
   // mask where applicable.
   assign clk_mask = /*clk &*/ ((clk_next & (~ui_in[6] | step_do)) | rst_or_update);
   
   // Determine current selection, linear address
   assign select_linear = {select_y, select_x};
   
   // Determine highlight selection color
   assign color_sel = color_fg + {{FIELD_COLOR - 1{1'b0}}, 1'b1};
   
   // Determine the current cell value for drawing.
   // The playing field is centered. For the selected cell, the X and Y
   // extreme pixels should be highlighted. All other pixels get normal
   // foreground/background coloring.
   assign vga_x_off  = vga_x - FIELD_PIX_DX;
   assign vga_x_ok   = (vga_x >= FIELD_PIX_DX) & (vga_x < (FIELD_PIX_DX + FIELD_PIX_LEN));
   assign vga_y_off  = vga_y - FIELD_PIX_DY;
   assign vga_y_ok   = (vga_y >= FIELD_PIX_DY) & (vga_y < (FIELD_PIX_DY + FIELD_PIX_LEN));
   assign vga_linear = {vga_y_off[FIELD_PIX_POW2 + FIELD_DEPTH - 1:FIELD_PIX_POW2],
                        vga_x_off[FIELD_PIX_POW2 + FIELD_DEPTH - 1:FIELD_PIX_POW2]};
   assign box_sel    = (vga_x_off[FIELD_PIX_POW2 - 1:0] == {FIELD_PIX_POW2{1'b0}}) |
                       (vga_x_off[FIELD_PIX_POW2 - 1:0] == {FIELD_PIX_POW2{1'b1}}) |
                       (vga_y_off[FIELD_PIX_POW2 - 1:0] == {FIELD_PIX_POW2{1'b0}}) |
                       (vga_y_off[FIELD_PIX_POW2 - 1:0] == {FIELD_PIX_POW2{1'b1}});
   assign cell_draw_orig = set_current[vga_linear];
   assign color_index    = (~vga_x_ok | ~vga_y_ok) ? 4'd3 :
                           ((vga_linear == select_linear) & box_sel) ? color_sel :
                           cell_draw_orig ? color_fg : color_bg;

   // ----- Register Logic -----
   
   /**
    * Primary registered process for fast clock updates for
    * the MCU. Intended primarily for reset timing.
    * @param[in] clk The fast register clock.
    */
   always @(posedge clk) begin : field_updates
      
      // Initialize all signals on reset
      if (rst_master) begin
      
         // Conway grid values
         set_update    <= {FIELD_LENGTH{1'b0}};
         rst_or_update <= 1'b1;
         count_psrng   <= {FIELD_DEPTH_2{1'b0}};
         
         // Vertical sync and clocking
         count_sync    <= 4'd0;
         clk_next      <= 1'b0;
         v_sync_update <= 1'b0;
      
         // Default colors
         color_fg <= 4'd10; // Teal
         color_bg <= 4'd3;  // Black
         
         // Update selection
         select_x <= {FIELD_DEPTH{1'b0}};
         select_y <= {FIELD_DEPTH{1'b0}};
         
         // Rising edge detection
         step_val      <= 1'b0;
         step_do       <= 1'b0;
         advance_fg    <= 1'b0;
         advance_bg    <= 1'b0;
         update_select <= 1'b0;
         update_val    <= 1'b0;
         update_random <= 1'b0;
      
      // Otherwise update the register state
      end else begin
      
         // Update vertical sync count when syncs occur.
         v_sync_update <= v_sync;
         count_sync    <= count_sync;
         clk_next      <= 1'b0;
         if (v_sync & ~v_sync_update & (count_sync >= ui_in[5:2])) begin
            clk_next   <= 1'b1;
            count_sync <= 4'd0;
         end else if (v_sync & ~v_sync_update) begin
            count_sync <= count_sync + 4'd1;
         end
      
         // Perform grid values updates where necessary.
         // By default, save the last seen value.
         // On updates, perform that specific kind of update.
         set_update    <= set_current;
         if (rst_or_update & update_random) begin
            set_update              <= set_update;
            set_update[count_psrng] <= psrng_data[0];
         end else if (rst_or_update) begin
            set_update                <= set_update;
            set_update[select_linear] <= uio_in[4];
         end
         
         // Latch update or randomize requests. All of them are given
         // a full "randomize update" time to complete, regardless of
         // which was requested.
         count_psrng   <= count_psrng;
         update_val    <= update_val;
         update_random <= update_random;
         rst_or_update <= rst_or_update;
         if (~update_val & (uio_in[5] | uio_in[6])) begin
            rst_or_update <= 1'b1;
            update_val    <= 1'b1;
            update_random <= uio_in[6] & ~uio_in[5];
            count_psrng   <= (uio_in[6] & ~uio_in[5]) ?
                             {FIELD_DEPTH_2{1'b0}} : {{FIELD_DEPTH_2 - 1{1'b1}}, 1'b0};
         end else if (update_val & ~uio_in[5] & ~uio_in[6]) begin
            update_val    <= 1'b0;
         end else if (rst_or_update & (count_psrng < {FIELD_DEPTH_2{1'b1}})) begin
            count_psrng   <= count_psrng + {{FIELD_DEPTH_2 - 1{1'b0}}, 1'b1};
         end else if (rst_or_update) begin
            rst_or_update <= 1'b0;
         end
            
         // Latch edges of the external step input to determine when to
         // do step updates.
         if (~step_val & ui_in[7]) begin
            step_val <= 1'b1;
            step_do  <= 1'b1;
         end else if (step_val & ~ui_in[7]) begin
            step_val <= 1'b0;
            step_do  <= 1'b0;
         end else if (~clk_next) begin
            // Hold steps when not actually ready for them
            step_val <= step_val;
            step_do  <= step_do;
         end else begin
            step_val <= step_val;
            step_do  <= 1'b0;
         end
         
         // Latch edges of the FG advance and advance color.
         color_fg   <= color_fg;
         advance_fg <= advance_fg;
         if (~advance_fg & ui_in[0]) begin
            advance_fg <= 1'b1;
            color_fg   <= color_fg + 4'd1;
         end else if (advance_fg & ~ui_in[0]) begin
            advance_fg <= 1'b0;
         end
         
         // Latch edges of the BG advance and advance color.
         color_bg   <= color_bg;
         advance_bg <= advance_bg;
         if (~advance_bg & ui_in[1]) begin
            advance_bg <= 1'b1;
            color_bg   <= color_bg + 4'd1;
         end else if (advance_bg & ~ui_in[1]) begin
            advance_bg <= 1'b0;
         end
         
         // Latch updates to the selection.
         update_select <= update_select;
         select_x      <= select_x;
         select_y      <= select_y;
         if (~update_select & uio_in[0]) begin // Up
            select_y      <= select_y - {{FIELD_DEPTH - 1{1'b0}}, 1'b1};
            update_select <= 1'b1;
         end else if (~update_select & uio_in[1]) begin // Down
            select_y      <= select_y + {{FIELD_DEPTH - 1{1'b0}}, 1'b1};
            update_select <= 1'b1;
         end else if (~update_select & uio_in[2]) begin // Left
            select_x      <= select_x - {{FIELD_DEPTH - 1{1'b0}}, 1'b1};
            update_select <= 1'b1;
         end else if (~update_select & uio_in[3]) begin // Right
            select_x      <= select_x + {{FIELD_DEPTH - 1{1'b0}}, 1'b1};
            update_select <= 1'b1;
         end else if (~update_select & uio_in[7]) begin // Origin
            select_x      <= {FIELD_DEPTH{1'b0}};
            select_y      <= {FIELD_DEPTH{1'b0}};
            update_select <= 1'b1;
         end else if (update_select & ~uio_in[0] & ~uio_in[1] &
                      ~uio_in[2] & ~uio_in[3] & ~uio_in[7]) begin
            update_select <= 1'b0;
         end
      end
   end

   // ----- Sub-Modules -----
   
   /** Primary Conway's Game of Life grid.*/
   ConwayGrid#(.WIDTH(2 ** FIELD_DEPTH), .HEIGHT(2 ** FIELD_DEPTH))
              core_grid(.clk(clk_mask), .rst(rst_or_update), .wrap(1'b1), .set(set_update),
                        .rule_born(`CONWAY_BORN), .rule_survive(`CONWAY_SURVIVE),
                        .cells(set_current));

   /** 640x480 standard VGA controller. Assumes 25.175MHz clock.*/
   ControllerVGA
              vga_out  (.clk(clk), .rst(rst_master),
                        .h_pos(vga_x), .v_pos(vga_y),
                        .h_sync(uo_out[3]), .h_blank(), .v_sync(v_sync), .v_blank());

   /** PSRNG from an alternating sequence generator for "good enough" field randomization.*/
   AlternatingStepGenerator#(.WIDTH(8), .GALOIS(1))
              psrng_asg(.clk(clk), .rst(rst_master), .en(1'b1),
                        .taps0(8'hE1), .taps1(8'h36), .taps2(8'hFA),
                        .def0(8'h23), .def1(8'h42), .def2(8'h69),
                        .data(psrng_data));

// ----- End of Module -----
endmodule

/* Helper Modules ********************************************************** */

/**
 * N-bit active bit counter.
 * Counts the number of active bits in the vector.
 * @note Supports all bit widths.
 * @tparam[in] WIDTH The bit width N.
 * @param[in] a The N-bit input value.
 * @param[out] res An N-bit value indicating the active bit count.
 * @note 2024-05-06 Tested bit-widths:
 *       Full:   1-8, 11, 15, 16
 *       Random: 24, 32, and 64; 2 ** 18 random values covered
 */
module ActiveBitCount#(parameter WIDTH = 32)
                      (input [WIDTH - 1:0] a, output [WIDTH - 1:0] res);

   // ----- Constants -----
   localparam LOGWIDTH  = $clog2(WIDTH);   /**< Base-2 log of bit width, for generic hookup.*/
   localparam FULLWIDTH = (2 ** LOGWIDTH); /**< Full width after power of two extension.*/

   // ----- Local Variables -----
   genvar pair_cnt;                    /**< Pair counter for generation loops.*/
   genvar gen_cnt;                     /**< Generation counter for generation loops.*/
   wire [FULLWIDTH - 1:0] a_fix; /**< Input extended to the nearest power of 2.*/
   wire [FULLWIDTH - 1:0] cnt[0:LOGWIDTH-1]; /**< Packed count for each generation.*/

   // ----- Combinatorial Logic -----

   // Generate the bit counting...
   generate
      
      // Input is already a power of two, no need for extension.
      if (FULLWIDTH == WIDTH) begin
         assign a_fix = a;
      
      // Input is not a power of two, extend to the nearest power of two.
      // Extension uses zeroes to not inflate the final result.
      end else begin
         assign a_fix[FULLWIDTH - 1:WIDTH] = {FULLWIDTH - WIDTH{1'b0}};
         assign a_fix[WIDTH - 1:0] = a;
      end

      // For the first power of two generation, add bits from the fixed input.
      // Because the initial value was fixed to a power of two, there are no possible
      // hanging bits in this calculation.
      // Assignment follows this pattern...
      //   cnt[0][1,3,5,7...:0,2,4,6...] = {1'b0, a_fix[0,2,4,6...]} + {1'b0, a_fix[1,3,5,7...]};
      for (pair_cnt = 0; pair_cnt < 2 ** (LOGWIDTH - 1); pair_cnt = pair_cnt + 1) begin : cnt_init
         assign cnt[0][`SUM_MSB(0, pair_cnt):`SUM_LSB(0, pair_cnt)] =
            {1'b0, a_fix[`LEFT_LSB(0, pair_cnt)]} +
            {1'b0, a_fix[`RIGHT_LSB(0, pair_cnt)]};
         // assign cnt[0][((pair_cnt + 1) * 2) - 1:pair_cnt * 2] =
         //    {1'b0, a_fix[pair_cnt * 2]} +
         //    {1'b0, a_fix[((pair_cnt + 1) * 2) - 1]};
      end

      // Iterate over each power of two generation beyond the first...
      for (gen_cnt = 1; gen_cnt < LOGWIDTH; gen_cnt = gen_cnt + 1) begin : cnt_pow_2

         // Iterate over each pair last generation sums, and add them together.
         // This is more complicated than the generic "for loop adding bits" approach,
         // but is purely combinatorial.
         // Because the initial value was fixed to a power of two, there are no possible
         // hanging bits in this calculation.
         // Assignment follows this pattern... Note that only (gen_cnt + 2) bits are needed for
         // each running sum.
         //   cnt[1][2,5,8...:0,3,6...] = {1'b0, cnt[0][1,5,9...:0,4,8...]} + {1'b0, cnt[0][3,7,11...:2,6,10...]};
         //   cnt[2][3,7,11...:0,4,8...] = {1'b0, cnt[1][2,8,14...:0,6,12...]} + {1'b0, cnt[0][5,11,17...:3,9,15...]};
         for (pair_cnt = 0; pair_cnt < 2 ** (LOGWIDTH - (gen_cnt + 1)); pair_cnt = pair_cnt + 1) begin : cnt_bit
            assign cnt[gen_cnt][`SUM_MSB(gen_cnt, pair_cnt):`SUM_LSB(gen_cnt, pair_cnt)] =
               {1'b0, cnt[gen_cnt - 1][`LEFT_MSB(gen_cnt, pair_cnt):`LEFT_LSB(gen_cnt, pair_cnt)]} +
               {1'b0, cnt[gen_cnt - 1][`RIGHT_MSB(gen_cnt, pair_cnt):`RIGHT_LSB(gen_cnt, pair_cnt)]};
            // assign cnt[gen_cnt][((pair_cnt + 1) * (gen_cnt + 2)) - 1:pair_cnt * (gen_cnt + 2)] =
            //    {1'b0, cnt[gen_cnt - 1][(((2 * pair_cnt) + 1) * (gen_cnt + 1)) - 1:(2 * pair_cnt) * (gen_cnt + 1)]} +
            //    {1'b0, cnt[gen_cnt - 1][(((2 * pair_cnt) + 2) * (gen_cnt + 1)) - 1:((2 * pair_cnt) + 1) * (gen_cnt + 1)]};
         end
      end

      // The output is in the last power of two's generation's data.
      // For outputs of N>3 bits, need to extend the output with zeroes.
      if (WIDTH > LOGWIDTH + 1) begin
         assign res[WIDTH - 1:LOGWIDTH + 1] = {WIDTH - LOGWIDTH - 1{1'b0}};
         assign res[LOGWIDTH:0] = cnt[LOGWIDTH - 1][LOGWIDTH:0];

      // For N==1 bits, output the input.
      end else if (WIDTH == 1) begin
         assign res = a;
      
      // For N<=3 bits, just output the full result.
      end else begin
         assign res = cnt[LOGWIDTH - 1];
      end
      
   endgenerate

   // ----- Register Logic -----

   // ----- Sub-Modules -----

// ----- End of Module -----
endmodule

/**
 * N-bit linear feedback shift register, with both Fibonacci and
 * Galois modes.
 * @tparam[in] WIDTH The width of data stored in the shift register.
 * @tparam[in] GALOIS If non-zero, uses Galois mode. Otherwise, uses
 * Fibonacci mode.
 * @param[in] clk The system clock.
 * @param[in] rst The system reset. Resetting the module reloads the
 * default value.
 * @param[in] en Enables the shift register to step to the next value.
 * @param[in] taps Which taps to enable for the shift register. Note
 * that the MSB is always treated as a 1, regardless of actual value.
 * @param[in] def The default value, used on reset.
 * @param[out] data The most recent output data.
 * @note 2024-06-25 Tested bit-widths:
 *       Full:   1-13 (including manual validation of m-sequence count)
 *       This module takes very significant time to test for N >= 12 due to
 *       the worst case of running every bit value for every tap value,
 *       leading to O(n^2) runtime.
 */
module LinearFeedbackShiftRegister#(parameter WIDTH = 32, parameter GALOIS = 0)
                     (input clk, input rst, input en,
                     input [WIDTH - 1:0] taps, input [WIDTH - 1:0] def,
                     output [WIDTH - 1:0] data);
   
   // ----- Constants -----
   localparam WIDTH_LESS_SAFE = (WIDTH == 1) ? 0 : WIDTH - 2; /**< Safe (WIDTH - 2) value.*/

   // ----- Local Variables -----
   integer i; /**< Loop count integer.*/
   reg  [WIDTH - 1:0] data_internal; /**< Shift data register.*/
   
   // ----- Combinatorial Logic -----
   
   // Assign outputs
   assign data = data_internal;

   // ----- Register Logic -----
   
   /**
    * Primary registered process for the LFSR.
    * Handles register updates.
    * @param[in] clk The register clock.
    */
   always @(posedge clk) begin : count_registers
      
      // Initialize to default value on reset
      if (rst) begin
         data_internal <= def;
      
      // Otherwise update the register state
      end else if (en) begin
         
         // Galois mode for register updates
         if (GALOIS > 0) begin
         
            // Wrap the final bit around to the first bit.
            // Galois mode: Treat as any other bit, less the final bit mask.
            data_internal[0] <= data_internal[WIDTH - 1];
            
            // Update all values beyond the initial bit.
            // Galois mode: each bit can be modified by the taps.
            for (i = 1; i < WIDTH; i = i + 1) begin
               data_internal[i] <= data_internal[i - 1] ^ (taps[WIDTH - 1 - i] & data_internal[WIDTH - 1]);
            end
            
         // Fibonacci mode for register updates
         end else begin
         
            // Wrap the final bit around to the first bit.
            // Fibonacci mode: XOR all tapped bits together for this.
            data_internal[0] <= ^(data_internal & {1'b1, taps[WIDTH_LESS_SAFE:0]});
            
            // Update all values beyond the initial bit.
            // Fibonacci mode: simple shift register.
            for (i = 1; i < WIDTH; i = i + 1) begin
               data_internal[i] <= data_internal[i - 1];
            end
         end
      
      // Not enabled, hold value
      end else begin
         data_internal <= data_internal;
      end
   end

   // ----- Sub-Modules -----

// ----- End of Module -----
endmodule

/**
 * N-bit alternating step generator based on three linear feedback
 * shift registers.
 * @tparam[in] WIDTH The width of data stored in the generator.
 * @tparam[in] GALOIS If non-zero, uses Galois mode. Otherwise, uses
 * Fibonacci mode.
 * @param[in] clk The system clock.
 * @param[in] rst The system reset. Resetting the module reloads the
 * default value.
 * @param[in] en Enables the shift register to step to the next value.
 * @param[in] taps0 Which taps to enable for the shift register. Note
 * that the MSB is always treated as a 1, regardless of actual value.
 * @param[in] taps1 Which taps to enable for the shift register. Note
 * that the MSB is always treated as a 1, regardless of actual value.
 * @param[in] taps2 Which taps to enable for the shift register. Note
 * that the MSB is always treated as a 1, regardless of actual value.
 * @param[in] def0 The default value, used on reset.
 * @param[in] def1 The default value, used on reset.
 * @param[in] def2 The default value, used on reset.
 * @param[out] data The most recent output data.
 * @note 2024-06-26 Tested bit-widths:
 *       Full:   3-16
 * Note that bit widths 1-2 were tested and failed. This is expected,
 * as those bit widths have no valid m-sequences to allow for useful
 * ASG sequence generation. "Successfully failed," one might say.
 */
module AlternatingStepGenerator#(parameter WIDTH = 32, parameter GALOIS = 0)
                     (input clk, input rst, input en,
                     input [WIDTH - 1:0] taps0, input [WIDTH - 1:0] taps1, input [WIDTH - 1:0] taps2,
                     input [WIDTH - 1:0] def0, input [WIDTH - 1:0] def1, input [WIDTH - 1:0] def2,
                     output [WIDTH - 1:0] data);
   
   // ----- Constants -----
   localparam WIDTH_LESS_SAFE = (WIDTH == 1) ? 0 : WIDTH - 2; /**< Safe (WIDTH - 2) value.*/

   // ----- Local Variables -----
   integer i;   /**< Loop count integer.*/
   wire    en0; /**< Enable for the first LFSR.*/
   wire    en1; /**< Enable for the second LFSR.*/
   wire [WIDTH - 1:0] data0;         /**< Shift data register for first LFSR.*/
   wire [WIDTH - 1:0] data1;         /**< Shift data register for second LFSR.*/
   wire [WIDTH - 1:0] data2;         /**< Shift data register for third LFSR.*/
   reg  [WIDTH - 1:0] data_internal; /**< Combined data output.*/
   
   // ----- Combinatorial Logic -----
   
   // Assign enables from via the controller LFSR's LSB.
   assign en0 = data2[0];
   assign en1 = ~data2[0];
   
   // Assign outputs.
   assign data = data_internal;

   // ----- Register Logic -----
   
   /**
    * Primary registered process for the alternating step generator.
    * Handles register updates.
    * @param[in] clk The register clock.
    */
   always @(posedge clk) begin : count_registers
      
      // Initialize to default value on reset
      if (rst) begin
         data_internal <= def2;
      
      // Otherwise update the register state
      end else if (en) begin
         
         // Combine peripheral LFSR LSBs for new LSB.
         data_internal[0] <= data0[0] ^ data1[0];
         
         // Shift all other bits up.
         for (i = 1; i < WIDTH; i = i + 1) begin
            data_internal[i] <= data_internal[i - 1];
         end
      
      // Not enabled, hold value
      end else begin
         data_internal <= data_internal;
      end
   end

   // ----- Sub-Modules -----
   
   /** First linear feedback shift register.*/
   LinearFeedbackShiftRegister #(.WIDTH(WIDTH), .GALOIS(GALOIS))
                     lsfr0 (.clk(clk), .rst(rst), .en(en0),
                     .taps(taps0), .def(def0), .data(data0));
   /** Second linear feedback shift register.*/
   LinearFeedbackShiftRegister #(.WIDTH(WIDTH), .GALOIS(GALOIS))
                     lsfr1 (.clk(clk), .rst(rst), .en(en1),
                     .taps(taps1), .def(def1), .data(data1));
   /** Third linear feedback shift register.*/
   LinearFeedbackShiftRegister #(.WIDTH(WIDTH), .GALOIS(GALOIS))
                     lsfr2 (.clk(clk), .rst(rst), .en(en),
                     .taps(taps2), .def(def2), .data(data2));

// ----- End of Module -----
endmodule

/**
 * A "standard" NTSC VGA controller. This gives a 640H by 480V resolution at
 * "60Hz" (actually 59.94Hz), and requires a pixel clock of 25.175 MHz.
 *
 * Pixel positions are given such that data can be addressed externally. As
 * such, only the syncing and blanking control signals are generated here
 * directly.
 * 
 * The following timings, given in pixels and in wall time, are followed:
 * + - + - - - - - - - - -+ - - - - - - - -+ - - - - - - - -+ - - - - - - - - + - - - - - - - - -+
 * |   |    Active Area   |   Front Porch  |      Sync      |    Back Porch   |       Total      |
 * + - + - - - - - - - - -+ - - - - - - - -+ - - - - - - - -+ - - - - - - - - + - - - - - - - - -+
 * | H | 640px (25.422us) | 16px (635.6ns) | 96px (3.813us) | 48px (1.9067us) | 800px ( 31.78us) |
 * | V | 480li (15.253ms) | 10li (317.8us) |  2li (63.56us) | 33li (1.0487ms) | 525li (16.683ms) |
 * + - + - - - - - - - - -+ - - - - - - - -+ - - - - - - - -+ - - - - - - - - + - - - - - - - - -+
 * 
 * @param[in] clk The pixel clock.
 * @param[in] rst Master reset.
 * @param[out] h_pos The current horizontal position, for data lookup.
 * @param[out] v_pos The current vertical position, for data lookup.
 * @param[out] h_sync The horizontal sync signal, asserted between the
 * horizontal porch segments.
 * @param[out] h_blank Indication that the signal is blanked due to being
 * out of the active horizontal drawing area. The rising edge of this signal
 * may be used as a "horizontal interrupt" to give a 6.3553us window before
 * the next horizontal line is drawn.
 * @param[out] v_sync The vertical sync signal, asserted between the
 * vertical porch segments.
 * @param[out] v_blank Indication that the signal is blanked due to
 * being out of the active vertical drawing area. The rising edge of this
 * signal may be used as a "vertical interrupt" to give a 1.430ms window
 * before the next frame is drawn.
 * @note Tested on 2024-08-09 Manually verified timings in table from waveform.
 */
module ControllerVGA(input clk, input rst,
                     output [`VGA_WIDTH - 1:0] h_pos, output [`VGA_WIDTH - 1:0] v_pos,
                     output h_sync, output h_blank, output v_sync, output v_blank);

   // ----- Constants -----
   localparam TIME_H_ACTIVE  = 640; /**< Horizontal active area, in pixels.*/
   localparam TIME_H_FRONT_P = 16;  /**< Horizontal front porch, in pixels.*/
   localparam TIME_H_SYNC    = 96;  /**< Horizontal sync period, in pixels.*/
   localparam TIME_H_BACK_P  = 48;  /**< Horizontal back porch, in pixels.*/
   localparam TIME_V_ACTIVE  = 480; /**< Vertical active area, in lines.*/
   localparam TIME_V_FRONT_P = 10;  /**< Vertical front porch, in lines.*/
   localparam TIME_V_SYNC    = 2;   /**< Vertical sync period, in lines.*/
   localparam TIME_V_BACK_P  = 33;  /**< Vertical back porch, in lines.*/
   localparam [`VGA_WIDTH - 1:0] TIME_ZERO = {`VGA_WIDTH{1'b0}};             /**< Sized constant of 0.*/
   localparam [`VGA_WIDTH - 1:0] TIME_ONE  = {{`VGA_WIDTH - 1{1'b0}}, 1'b1}; /**< Sized constant of 1.*/

   // ----- Local Variables -----
   reg  h_sync_int;  /**< Internal horizontal sync.*/
   reg  h_blank_int; /**< Internal horizontal blank.*/
   wire h_max_val;   /**< Horizontal line at max value (or reset).*/
   reg  v_sync_int;  /**< Internal vertical sync.*/
   reg  v_blank_int; /**< Internal vertical blank.*/
   wire v_max_val;   /**< Vertical frame at max value (or reset).*/
   reg  [`VGA_WIDTH - 1:0] h_pos_int; /**< Internal horizontal position count.*/
   reg  [`VGA_WIDTH - 1:0] v_pos_int; /**< Internal vertical position count.*/

   // ----- Combinatorial Logic -----
   
   // Determine when a line or frame is complete
   assign h_max_val = (h_pos_int == (TIME_H_ACTIVE + TIME_H_FRONT_P + TIME_H_SYNC + TIME_H_BACK_P - TIME_ONE));
   assign v_max_val = (v_pos_int == (TIME_V_ACTIVE + TIME_V_FRONT_P + TIME_V_SYNC + TIME_V_BACK_P - TIME_ONE));
   
   // Save output values
   assign h_blank = h_blank_int;
   assign v_blank = v_blank_int;
   assign h_sync  = h_sync_int;
   assign v_sync  = v_sync_int;
   assign h_pos   = h_pos_int;
   assign v_pos   = v_pos_int;

   // ----- Register Logic -----
   
   /**
   * Main process logic for VGA control.
   * @note Remember that a register is updated each clock cycle. If you want
   * an output to be true on clock cycle N, the calculation for it should
   * indicate so on clock cycle (N - 1), so that it latches appropriately.
   * For a range of [N, M), the calculation should check (x >= N - 1) and
   * (x < M - 1).
   * @param[in] clk The pixel clock
   */
   always @(posedge clk) begin

      // Reset all signals when the reset is active.
      if (rst) begin
         h_blank_int <= 1'b0;
         h_sync_int  <= 1'b0;
         h_pos_int   <= TIME_ZERO;
         v_blank_int <= 1'b0;
         v_sync_int  <= 1'b0;
         v_pos_int   <= TIME_ZERO;
      
      // Otherwise, normally update signals.
      end else begin

         // Determine when the horizontal signal is blanked.
         h_blank_int <= (h_pos_int >= TIME_H_ACTIVE - TIME_ONE) & ~h_max_val;

         // Determine when the horizontal sync signal is active.
         h_sync_int <= (h_pos_int >= (TIME_H_ACTIVE + TIME_H_FRONT_P - TIME_ONE)) &
                       (h_pos_int <  (TIME_H_ACTIVE + TIME_H_FRONT_P + TIME_H_SYNC - TIME_ONE));

         // Update the current horizontal position.
         h_pos_int <= (h_max_val) ? TIME_ZERO : (h_pos_int + TIME_ONE);
         
         // If the horizontal position is at its end, update vertical signals.
         if (h_max_val) begin

            // Determine when the vertical signal is blanked.
            v_blank_int <= (v_pos_int >= TIME_V_ACTIVE - TIME_ONE) & ~v_max_val;

            // Determine when the vertical sync signal is active.
            v_sync_int <= (v_pos_int >= (TIME_V_ACTIVE + TIME_V_FRONT_P - TIME_ONE)) &
                          (v_pos_int <  (TIME_V_ACTIVE + TIME_V_FRONT_P + TIME_V_SYNC - TIME_ONE));

            // Update the current vertical position.
            v_pos_int  <= (v_max_val) ? TIME_ZERO : (v_pos_int + TIME_ONE);
            
         // Otherwise keep the prior vertical signals.
         end else begin
            v_blank_int <= v_blank_int;
            v_sync_int  <= v_sync_int;
            v_pos_int   <= v_pos_int;
         end
      end
   end

   // ----- Sub-Modules -----

// ----- End of Module -----
endmodule

/**
 * Conway's Game of Life grid of arbitrary size. Has been generified to
 * allow for any birth/survival rule set.
 * @note Grid width and height must be greater than one. This is partially
 *       to simplify logic, and partly because a 1xH, Wx1, or 1x1 grid all
 *       have trivial behavior due to the lack of neighbors.
 * @tparam[in] WIDTH The width of the grid.
 * @tparam[in] HEIGHT The height of the grid.
 * @param[in] clk The system clock.
 * @param[in] rst The system reset.
 * @param[in] wrap Whether to wrap grid borders (0=none, 1=wrap).
 * @param[in] set Synchronous (re)set for each cell in the grid.
 * @param[in] rule_born Each bit indicates a number of neighbors for
 * which a cell is "born." The standard Conway rule is 0x04 (born only on
 * three neighbors). Does not count toward survival.
 * @param[in] rule_survive Each bit indicates a number of neighbors for
 * which a cell survives. The standard Conway rule is 0x06 (survives with
 * two or three neighbors).
 * @param[out] cells Current cell values.
 * @note 2024-09-26 Tested for 5x5 grid, visually, step-by-step
 *       for 32 steps, wrapping and not, with Conway rules and a few others
 */
module ConwayGrid#(parameter WIDTH = 5, parameter HEIGHT = 5)
                  (input clk, input rst, input wrap, input [(WIDTH * HEIGHT) - 1:0] set,
                  input [`CONWAY_WIDTH:0] rule_born, input [`CONWAY_WIDTH:0] rule_survive,
                  output [(WIDTH * HEIGHT) - 1:0] cells);
   
   // ----- Constants -----
   genvar i; /**< Generation counter for X-axis of cells.*/
   genvar j; /**< Generation counter for Y-axis of cells.*/
   integer k;
   wire [(WIDTH * HEIGHT) - 1:0] c; /**< Internal cell values.*/

   // ----- Local Variables -----

   // ----- Combinatorial Logic -----
   
   // Assign output values
   assign cells = c;

   // ----- Register Logic -----

   // ----- Sub-Modules -----
   
   // Generate all cells in the grid.
   // To allow for wrapping, the grid is broken up into corners, left,
   // right, top, bottom, and center segments.
   generate
      
      // === Corner cells ===
      /** Conway cell for the top-left entry of the grid.*/
      ConwayCell c_ul(.clk(clk), .rst(rst), .rule_born(rule_born), .rule_survive(rule_survive),
           .set(set[`C_ID(0, HEIGHT - 1)]), .val(c[`C_ID(0, HEIGHT - 1)]), .neighbor(
           {c[`C_ID(-1, HEIGHT    )] & wrap, c[`C_ID(0, HEIGHT    )] & wrap, c[`C_ID(1, HEIGHT    )] & wrap,
            c[`C_ID(-1, HEIGHT - 1)] & wrap,                                 c[`C_ID(1, HEIGHT - 1)],
            c[`C_ID(-1, HEIGHT - 2)] & wrap, c[`C_ID(0, HEIGHT - 2)],        c[`C_ID(1, HEIGHT - 2)]}));
      /** Conway cell for the top-right of the grid.*/
      ConwayCell c_ur(.clk(clk), .rst(rst), .rule_born(rule_born), .rule_survive(rule_survive),
           .set(set[`C_ID(WIDTH - 1, HEIGHT - 1)]), .val(c[`C_ID(WIDTH - 1, HEIGHT - 1)]), .neighbor(
           {c[`C_ID(WIDTH - 2, HEIGHT    )] & wrap, c[`C_ID(WIDTH - 1, HEIGHT    )] & wrap, c[`C_ID(WIDTH, HEIGHT    )] & wrap,
            c[`C_ID(WIDTH - 2, HEIGHT - 1)],                                                c[`C_ID(WIDTH, HEIGHT - 1)] & wrap,
            c[`C_ID(WIDTH - 2, HEIGHT - 2)],        c[`C_ID(WIDTH - 1, HEIGHT - 2)],        c[`C_ID(WIDTH, HEIGHT - 2)] & wrap}));
      /** Conway cell for the bottom-left of the grid.*/
      ConwayCell c_dl(.clk(clk), .rst(rst), .rule_born(rule_born), .rule_survive(rule_survive),
           .set(set[`C_ID(0, 0)]), .val(c[`C_ID(0, 0)]), .neighbor(
           {c[`C_ID(-1,  1)] & wrap, c[`C_ID(0,  1)],        c[`C_ID(1,  1)],
            c[`C_ID(-1,  0)] & wrap,                         c[`C_ID(1,  0)],
            c[`C_ID(-1, -1)] & wrap, c[`C_ID(0, -1)] & wrap, c[`C_ID(1, -1)] & wrap}));
      /** Conway cell for the bottom-right of the grid.*/
      ConwayCell c_dr(.clk(clk), .rst(rst), .rule_born(rule_born), .rule_survive(rule_survive),
           .set(set[`C_ID(WIDTH - 1, 0)]), .val(c[`C_ID(WIDTH - 1, 0)]), .neighbor(
           {c[`C_ID(WIDTH - 2,  1)],        c[`C_ID(WIDTH - 1,  1)],        c[`C_ID(WIDTH,  1)] & wrap,
            c[`C_ID(WIDTH - 2,  0)],                                        c[`C_ID(WIDTH,  0)] & wrap,
            c[`C_ID(WIDTH - 2, -1)] & wrap, c[`C_ID(WIDTH - 1, -1)] & wrap, c[`C_ID(WIDTH, -1)] & wrap}));
      
      // === Left cells ===
      for (j = 1; j < HEIGHT - 1; j = j + 1) begin : conway_left
         
         /** Conway cell for this left border entry of the grid.*/
         ConwayCell c_l(.clk(clk), .rst(rst), .rule_born(rule_born), .rule_survive(rule_survive),
              .set(set[`C_ID(0, j)]), .val(c[`C_ID(0, j)]), .neighbor(
              {c[`C_ID(-1, j + 1)] & wrap, c[`C_ID(0, j + 1)], c[`C_ID(1, j + 1)],
               c[`C_ID(-1, j    )] & wrap,                     c[`C_ID(1, j    )],
               c[`C_ID(-1, j - 1)] & wrap, c[`C_ID(0, j - 1)], c[`C_ID(1, j - 1)]}));
      end
      
      // === Right cells ===
      for (j = 1; j < HEIGHT - 1; j = j + 1) begin : conway_right
         
         /** Conway cell for this right border entry of the grid.*/
         ConwayCell c_r(.clk(clk), .rst(rst), .rule_born(rule_born), .rule_survive(rule_survive),
              .set(set[`C_ID(WIDTH - 1, j)]), .val(c[`C_ID(WIDTH - 1, j)]), .neighbor(
              {c[`C_ID(WIDTH - 2, j + 1)], c[`C_ID(WIDTH - 1, j + 1)], c[`C_ID(WIDTH, j + 1)] & wrap,
               c[`C_ID(WIDTH - 2, j    )],                             c[`C_ID(WIDTH, j    )] & wrap,
               c[`C_ID(WIDTH - 2, j - 1)], c[`C_ID(WIDTH - 1, j - 1)], c[`C_ID(WIDTH, j - 1)] & wrap}));
      end
      
      // === Top cells ===
      for (i = 1; i < WIDTH - 1; i = i + 1) begin : conway_bottom
         
         /** Conway cell for this top entry of the grid.*/
         ConwayCell c_u(.clk(clk), .rst(rst), .rule_born(rule_born), .rule_survive(rule_survive),
              .set(set[`C_ID(i, HEIGHT - 1)]), .val(c[`C_ID(i, HEIGHT - 1)]), .neighbor(
              {c[`C_ID(i - 1, HEIGHT    )] & wrap, c[`C_ID(i, HEIGHT    )] & wrap, c[`C_ID(i + 1, HEIGHT    )] & wrap,
               c[`C_ID(i - 1, HEIGHT - 1)],                                        c[`C_ID(i + 1, HEIGHT - 1)],
               c[`C_ID(i - 1, HEIGHT - 2)],        c[`C_ID(i, HEIGHT - 2)],        c[`C_ID(i + 1, HEIGHT - 2)]}));
      end
      
      // === Bottom cells ===
      for (i = 1; i < WIDTH - 1; i = i + 1) begin : conway_top
         
         /** Conway cell for this bottom entry of the grid.*/
         ConwayCell c_d(.clk(clk), .rst(rst), .rule_born(rule_born), .rule_survive(rule_survive),
              .set(set[`C_ID(i, 0)]), .val(c[`C_ID(i, 0)]), .neighbor(
              {c[`C_ID(i - 1,  1)],        c[`C_ID(i,  1)],        c[`C_ID(i + 1,  1)],
               c[`C_ID(i - 1,  0)],                                c[`C_ID(i + 1,  0)],
               c[`C_ID(i - 1, -1)] & wrap, c[`C_ID(i, -1)] & wrap, c[`C_ID(i + 1, -1)] & wrap}));
      end
      
      // === Center cells ===
      for (i = 1; i < WIDTH - 1; i = i + 1) begin : conway_center_x
         for (j = 1; j < HEIGHT - 1; j = j + 1) begin : conway_center_y
         
            /** Conway cell for this center entry of the grid.*/
            ConwayCell c_c(.clk(clk), .rst(rst), .rule_born(rule_born), .rule_survive(rule_survive),
                 .set(set[`C_ID(i, j)]), .val(c[`C_ID(i, j)]), .neighbor(
                 {c[`C_ID(i - 1, j + 1)], c[`C_ID(i, j + 1)], c[`C_ID(i + 1, j + 1)],
                  c[`C_ID(i - 1, j    )],                     c[`C_ID(i + 1, j    )],
                  c[`C_ID(i - 1, j - 1)], c[`C_ID(i, j - 1)], c[`C_ID(i + 1, j - 1)]}));
         end
      end
   endgenerate

   // Generate block to cause an error on invalid grid sizes.
   generate
      if ((WIDTH <= 1) || (HEIGHT <= 1)) begin : error_width
         IllegalParameterWidthTooSmall non_existing_module();
      end
   endgenerate

// ----- End of Module -----
endmodule

/**
 * Simple Conway's Game of Life cell.
 * Uses a population count and some basic logic to determine cell state.
 * @param[in] clk The system clock.
 * @param[in] rst The system reset.
 * @param[in] set Synchronous reset state.
 * @param[in] rule_born Each bit indicates a number of neighbors for
 * which a cell is "born." The standard Conway rule is 0x008 (born only on
 * three neighbors). Does not count toward survival.
 * @param[in] rule_survive Each bit indicates a number of neighbors for
 * which a cell survives. The standard Conway rule is 0x00C (survives with
 * two or three neighbors).
 * @param[in] neighbor Neighboring cell values.
 * @param[out] val The current cell value.
 * @note Tested as part of the larger Conway Grid module.
 */
module ConwayCell(input clk, input rst, input set,
                  input [`CONWAY_WIDTH:0] rule_born, input [`CONWAY_WIDTH:0] rule_survive,
                  input [`CONWAY_WIDTH - 1:0] neighbor, output val);

   // ----- Constants -----

   // ----- Local Variables -----
   wire born;         /**< Born rule passed.*/
   wire survive;      /**< Survive rule passed.*/
   wire next;         /**< Next cell value.*/
   reg  val_internal; /**< Internal cell value.*/
   wire [`CONWAY_WIDTH - 1:0] count; /**< Alive neighbor cell count.*/

   // ----- Combinatorial Logic -----
   
   // Conway's Game of Life cell survival rules:
   //   * Cells are born if they have 3 neighbors.
   //   * Cells survive if they are alive and have 2 or 3 neighbors.
   //     Simple optimization on 3 neighbors due to birth rule.
   //   * Cell dies or stays dead if not born or surviving.
   assign born    = rule_born[count[`CONWAY_COUNT - 1:0]];
   assign survive = rule_survive[count[`CONWAY_COUNT - 1:0]];
   assign next    = (born & ~val_internal) | (survive & val_internal);
   assign val     = val_internal;

   // ----- Register Logic -----

   /**
    * Primary registered process for the Conway Cell module.
    * Handles cell state updates.
    * @param[in] clk The register clock.
    */
   always @(posedge clk) begin : conway_cell_registers
      
      // Initialize to default values on reset
      if (rst) begin
         val_internal <= set;
      
      // Otherwise update the cell state
      end else begin
         val_internal <= next;
      end
   end

   // ----- Sub-Modules -----
   
   /** Active bit counter as the core portion of the Game of Life cell.*/
   ActiveBitCount #(.WIDTH(`CONWAY_WIDTH))
                  cell_count(.a(neighbor), .res(count));

// ----- End of Module -----
endmodule

/* End of File ************************************************************* */
