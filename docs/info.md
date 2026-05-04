<!---
This file is used to generate your project datasheet. Please fill in the information below and delete any unused sections.

You can also include images in this folder and reference them in the markdown. Each image must be less than 512 kb in size, and the combined size of all images must be less than 1 MB.
-->

## desc
quick and dirty repo at the LatchUp Conference, will refurbish repo for a proper submission.

## minimized MIPS-adjacent OCPU instruction set
the instruction set of the minimized MIPS-adjacent cpu is as follows:

0000 ldi imm      ; load immediate value into accumulator (a = imm)
0001 lda addr     ; load accumulator from memory (a = memory[addr])
0010 sta addr     ; store accumulator to memory (memory[addr] = a)
0011 lda [addr]   ; load accumulator from indirect address (a = memory[memory[addr]])
0100 sta [addr]   ; store accumulator to indirect address (memory[memory[addr]] = a)
0101 add addr     ; add memory to accumulator (a = a + memory[addr])
0110 adc addr     ; add memory and carry to accumulator (a = a + memory[addr] + carry)
0111 nand addr    ; bitwise nand accumulator and memory (a = ~(a & memory[addr]))
1000 shr          ; shift accumulator right by 1 bit
1001 jmp addr     ; jump to address
1010 jz addr      ; jump if accumulator is zero
1011 jc addr      ; jump if carry flag is set
1100 call addr    ; call subroutine (push pc, jump to addr)
1101 ret          ; return from subroutine (pop pc)
1110 push         ; push accumulator to stack
1111 pop          ; pop from stack to accumulator

## minimized 6502 OCPU instruction set (CISC)
this 8-bit opcode instruction set is heavily paired-down but explicitly mapped to the core 6502 architecture. by including the x and y registers in hardware, we natively support the 6502's indexed addressing modes, which are heavily used by C compilers for arrays and pointers.

memory & immediate operations:
- `lda #imm` / `ldx #imm` / `ldy #imm`  ; load immediate (a/x/y = imm)
- `lda addr` / `ldx addr` / `ldy addr`  ; load from memory
- `sta addr` / `stx addr` / `sty addr`  ; store to memory
- `lda addr,x` / `sta addr,x`           ; absolute x-indexed (target = addr + x)
- `lda (addr),y` / `sta (addr),y`       ; indirect y-indexed (target = memory[addr] + y)

alu (math & logic):
- `adc addr`     ; add with carry (a = a + memory[addr] + c)
- `sbc addr`     ; subtract with carry (a = a - memory[addr] - !c)
- `and addr`     ; bitwise and (a = a & memory[addr])
- `eor addr`     ; exclusive or (a = a ^ memory[addr])
- `ora addr`     ; bitwise or (a = a | memory[addr])
- `asl`          ; arithmetic shift left (shifts accumulator, pushes MSB to carry)
- `lsr`          ; logical shift right (shifts accumulator, pushes LSB to carry)
- `inx` / `dex`  ; increment / decrement x
- `iny` / `dey`  ; increment / decrement y

register transfers:
- `tax` / `txa`  ; transfer a to x / x to a
- `tay` / `tya`  ; transfer a to y / y to a

status flags & control:
- `sec` / `clc`  ; set / clear carry flag
- `sei` / `cli`  ; set / clear interrupt disable

control flow & subroutines:
- `jmp addr`     ; unconditional jump
- `beq addr`     ; branch on result zero (zero flag set)
- `bne addr`     ; branch on not zero (zero flag clear)
- `bcs addr`     ; branch on carry set
- `bcc addr`     ; branch on carry clear
- `jsr addr`     ; jump to subroutine (pushes PC to stack)
- `rts`          ; return from subroutine (pops PC)
- `rti`          ; return from interrupt (pops SR, then PC)
- `pha` / `pla`  ; push accumulator / pull accumulator

## datapath architecture
the ocpu features a dual-core architectural approach utilizing a multi-level fsm hierarchy to manage execution and synchronization:
- master fsm: a top-level controller responsible for issuing global states such as `run`, `halt`, or `simd`. it dictates whether the cores run independently or operate in lockstep.
- internal core fsms: each core (core 0 and core 1) possesses its own local multi-cycle fsm. when in standard executing modes, these handle the independent fetch-decode-execute loops.
- simd execution: when the master supervisor engages simd mode, both core 0 and core 1 synchronize to execute identical instruction bytes fetched from memory simultaneously. rather than hardcoding memory banks in hardware, data divergence is achieved using the index registers (`x` and `y`). by initializing each core's index registers to different offsets before entering simd mode, a single instruction like `lda array,x` allows each core to natively access different data elements simultaneously.
- accumulator-based logic: to severely constrain the flip-flop footprint required per core, the datapath relies purely on an accumulator and strictly defined index registers (x, y) rather than a generalized register file.

## features
- the programmer-visible registers include an 8-bit accumulator (a), index registers (x, y), and an 8-bit stack pointer (sp).
- the internal datapath consists of a program counter (pc), instruction register (ir), and memory data register (mdr). note that the pc is 16-bit, allowing standard 64kb addressability natively.
- the peripheral registers include interrupt vector and enable registers. to securely access mbits of external memory beyond the standard 64kb address space without complicating external peripheral logic, a zero-page memory-mapped i/o (mmio) banking register is used. writing to address `0xff` (e.g., `sta $ff`) inherently flips the upper memory lines sent from the cpu out to the external serial memory, maintaining hardware simplicity and 100% isa compatibility with standard 6502 compilers.
- the controllable target pll behaves independently so the cpu clock speed can be dynamically governed externally to control power draw and test frequency bounds.

