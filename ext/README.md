// External FPGA Modules
// =====================
// This folder contains Verilog code for the external FPGA that communicates
// with the tt26-ocpu ASIC via the OSPI (8-bit parallel) slave interface.

// OSPI Protocol (ASIC is slave, external FPGA is master)
// ======================================================
// The external FPGA drives the OSPI interface to:
//   1. Load instruction pages into iRAM (16 instructions per page)
//   2. Monitor the page_interrupt flag (CPU reached PC==15)
//   3. Coordinate page transitions via page_done handshake
//
// Pin Mapping (from ASIC perspective):
//   uio_in[0]   = SCK (serial clock from master)
//   uio_in[1]   = CS_N (chip select from master, active low)
//   uio_in[7:2] = IO_I (data in, from master to ASIC)
//   uio_out[7:0] = IO_O (data out, from ASIC to master)
//   uio_oe[7:0]  = output enable (ASIC drives when OSPI active)
//
//   uo_out[0] = page_interrupt (pulse: CPU needs next page)
//   uo_out[1] = page_loading_ack (flag: CPU waiting for page)
//   uo_out[2] = cpu_halted (flag: CPU halted)
//
//   uio_in[3] = page_done (from FPGA to ASIC: new page loaded, resume)
//   uio_in[4] = page_loading (from FPGA to ASIC: currently loading page)

// OSPI Command Bytes
// ==================
//   0x02 = WRITE (cmd + 3 addr + 1 data)
//   0x03 = READ  (cmd + 3 addr, then read data on next byte)

// Address Space
// =============
//   0x00XX00 = iRAM slot XX (where XX = instruction index 0-15)
//   0xFF0000 = page register (write only)

// Transaction Format
// ==================
// Master transmits 5 bytes per transaction:
//   Byte 0: Command (0x02 for write)
//   Byte 1: Address[23:16] (always 0x00 for iRAM)
//   Byte 2: Address[15:8] (instruction index)
//   Byte 3: Address[7:0] (always 0x00)
//   Byte 4: Data (instruction byte, 17 bits but only lower 8 used in this version)
//
// Paging Protocol
// ===============
// 1. CPU executes code from iRAM (16 instructions, PC 0-15)
// 2. When CPU reaches PC==15:
//    - page_interrupt asserts (pulse on uo_out[0])
//    - CPU halts
//    - page_loading_ack asserts (on uo_out[1])
// 3. External FPGA sees page_interrupt and page_loading_ack
// 4. FPGA asserts page_loading on uio_in[4] (tells CPU: loading in progress)
// 5. FPGA loads 16 instructions via OSPI transactions:
//    for i in 0..15:
//      send WRITE command to addr 0x00[i]00 with instruction data
// 6. FPGA asserts page_done on uio_in[3] (tells CPU: new page ready, resume)
// 7. CPU resumes from PC==0 with new page loaded

// Implementation Guidance
// =======================
// 1. Create an FSM that monitors page_interrupt from ASIC
// 2. When interrupted, set page_loading signal high
// 3. Fetch next 16 instructions from external storage (e.g., DRAM, SPI flash)
// 4. Send each via OSPI WRITE command
// 5. After all 16 loaded, pulse page_done to resume CPU
// 6. Increment page counter and wait for next interrupt

// Example: ospi_master.v provides a template for page loading coordination.
// Customize the instruction source (page_data_in) to match your storage system.
