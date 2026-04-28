// ============================================================
// riscv_multicycle.sv
// Multicycle RISC-V Processor – ELE432 Lab 3
// Based on Harris & Harris "Digital Design and Computer Architecture:
// RISC-V Edition" (Elsevier, 2021)
//
// Supported instructions: addi, add, sub, and, or, slt, lw, sw, beq, jal
//
// JAL implementation note:
//   At S0 (Fetch), OldPC ← current PC, then PC ← PC+4.
//   So after Fetch: OldPC = JAL_address, PC = JAL_address+4 (return addr).
//   At S1 (Decode), ALU computes OldPC+ImmJ → stored in ALUOut.
//   At S9 (JAL execute):
//     - JAL=1 overrides PC ← ALUOut (= jump target, computed in S1)
//     - ALU recomputes OldPC+4 (ALUSrcA=0, ALUSrcB=10)
//     - ResultSrc=10: Result = ALUResult = OldPC+4 = return address
//     - RegWrite=1: rd ← Result = return address  ✓
//
// Module hierarchy:
//   top → riscv → controller → maindec
//                            → aludec
//              → datapath
//      → mem
// ============================================================

// ============================================================
// TOP
// ============================================================
module top (
    input  logic        clk, reset,
    output logic [31:0] WriteData, DataAdr,
    output logic        MemWrite
);
    logic [31:0] ReadData;

    riscv cpu (
        .clk      (clk),
        .reset    (reset),
        .ReadData (ReadData),
        .MemWrite (MemWrite),
        .DataAdr  (DataAdr),
        .WriteData(WriteData)
    );

    mem memory (
        .clk (clk),
        .we  (MemWrite),
        .a   (DataAdr),
        .wd  (WriteData),
        .rd  (ReadData)
    );
endmodule

// ============================================================
// UNIFIED MEMORY  (instruction + data, word-addressed)
// 64 words × 32-bit = 256 bytes
// Initialized from memfile.txt at simulation start
// ============================================================
module mem (
    input  logic        clk, we,
    input  logic [31:0] a, wd,
    output logic [31:0] rd
);
    logic [31:0] RAM [63:0];
    initial $readmemh("memfile.txt", RAM);

    assign rd = RAM[a[31:2]];          // combinational read (word-aligned)

    always_ff @(posedge clk)
        if (we) RAM[a[31:2]] <= wd;    // synchronous write
endmodule

// ============================================================
// RISCV  – top-level processor (controller + datapath)
// ============================================================
module riscv (
    input  logic        clk, reset,
    input  logic [31:0] ReadData,
    output logic        MemWrite,
    output logic [31:0] DataAdr, WriteData
);
    // Controller → Datapath
    logic        PCWrite, AdrSrc, IRWrite, RegWrite, ALUSrcA;
    logic [1:0]  ALUSrcB, ResultSrc, ImmSrc;
    logic [2:0]  ALUControl;
    logic        JAL;       // 1: override PC ← ALUOut (jump target)

    // Datapath → Controller
    logic        Zero;
    logic [6:0]  op;
    logic [2:0]  funct3;
    logic        funct7b5;

    controller ctrl (
        .clk       (clk),      .reset    (reset),
        .op        (op),       .funct3   (funct3),
        .funct7b5  (funct7b5), .Zero     (Zero),
        .PCWrite   (PCWrite),  .AdrSrc   (AdrSrc),
        .MemWrite  (MemWrite), .IRWrite  (IRWrite),
        .ResultSrc (ResultSrc),.ALUSrcA  (ALUSrcA),
        .ALUSrcB   (ALUSrcB),  .ImmSrc   (ImmSrc),
        .ALUControl(ALUControl),.RegWrite(RegWrite),
        .JAL       (JAL)
    );

    datapath dp (
        .clk       (clk),      .reset    (reset),
        .ReadData  (ReadData),
        .PCWrite   (PCWrite),  .AdrSrc   (AdrSrc),
        .IRWrite   (IRWrite),  .RegWrite (RegWrite),
        .ALUSrcA   (ALUSrcA),  .ALUSrcB  (ALUSrcB),
        .ResultSrc (ResultSrc),.ImmSrc   (ImmSrc),
        .ALUControl(ALUControl),.JAL     (JAL),
        .Zero      (Zero),     .op       (op),
        .funct3    (funct3),   .funct7b5 (funct7b5),
        .Adr       (DataAdr),  .WriteData(WriteData)
    );
endmodule

// ============================================================
// CONTROLLER
// ============================================================
module controller (
    input  logic       clk, reset,
    input  logic [6:0] op,
    input  logic [2:0] funct3,
    input  logic       funct7b5,
    input  logic       Zero,
    output logic       PCWrite, AdrSrc, MemWrite, IRWrite,
                       RegWrite, ALUSrcA,
    output logic [1:0] ResultSrc, ALUSrcB, ImmSrc,
    output logic [2:0] ALUControl,
    output logic       JAL
);
    logic       Branch, PCUpdate;
    logic [1:0] ALUOp;

    maindec md (
        .clk      (clk),     .reset    (reset),
        .op       (op),
        .Branch   (Branch),  .PCUpdate (PCUpdate),
        .RegWrite (RegWrite),.MemWrite (MemWrite),
        .IRWrite  (IRWrite), .ResultSrc(ResultSrc),
        .ALUSrcA  (ALUSrcA), .ALUSrcB  (ALUSrcB),
        .ImmSrc   (ImmSrc),  .AdrSrc   (AdrSrc),
        .ALUOp    (ALUOp),   .JAL      (JAL)
    );

    aludec ad (
        .opb5      (op[5]),     .funct3   (funct3),
        .funct7b5  (funct7b5),  .ALUOp    (ALUOp),
        .ALUControl(ALUControl)
    );

    // PCWrite: unconditional update OR branch taken
    assign PCWrite = PCUpdate | (Branch & Zero);
endmodule

// ============================================================
// MAIN DECODER  (11-state Moore FSM)
//
//  S0  Fetch      S1  Decode
//  S2  MemAdr     S3  MemRead   S4  MemWB
//  S5  MemWrite
//  S6  ExecuteR   S7  ALUWB
//  S8  ExecuteI
//  S9  JAL
//  S10 BEQ
// ============================================================
module maindec (
    input  logic       clk, reset,
    input  logic [6:0] op,
    output logic       Branch, PCUpdate, RegWrite, MemWrite,
                       IRWrite, ALUSrcA, AdrSrc, JAL,
    output logic [1:0] ResultSrc, ALUSrcB, ImmSrc,
    output logic [1:0] ALUOp
);
    typedef enum logic [3:0] {
        S0=4'd0, S1=4'd1, S2=4'd2,  S3=4'd3,  S4=4'd4,
        S5=4'd5, S6=4'd6, S7=4'd7,  S8=4'd8,
        S9=4'd9, S10=4'd10
    } statetype;

    statetype state, nextstate;

    // ---- State register ----
    always_ff @(posedge clk or posedge reset)
        if (reset) state <= S0;
        else       state <= nextstate;

    // ---- Next-state logic ----
    always_comb begin
        case (state)
            S0:  nextstate = S1;
            S1:  case (op)
                     7'b0000011: nextstate = S2;
                     7'b0100011: nextstate = S2;
                     7'b0110011: nextstate = S6;
                     7'b0010011: nextstate = S8;
                     7'b1101111: nextstate = S9;
                     7'b1100011: nextstate = S10;
                     default:    nextstate = S0;
                 endcase
            S2:  if (op[5]) nextstate = S5; else nextstate = S3;
            S3:  nextstate = S4;
            S4:  nextstate = S0;
            S5:  nextstate = S0;
            S6:  nextstate = S7;
            S7:  nextstate = S0;
            S8:  nextstate = S7;
            S9:  nextstate = S0;
            S10: nextstate = S0;
            default: nextstate = S0;
        endcase
    end

    // ---- Output logic (Moore) ----
    always_comb begin
        // Safe defaults – all disabled / ADD operation
        IRWrite = 0; AdrSrc = 0;   ALUSrcA = 0;
        ALUSrcB = 2'b00;           ResultSrc = 2'b00;
        ImmSrc  = 2'b00;           ALUOp    = 2'b00;
        RegWrite = 0; MemWrite = 0;
        Branch = 0;  PCUpdate = 0; JAL = 0;

        case (state)
            // ------------------------------------------------
            // S0 FETCH
            //   Memory address = PC  (AdrSrc=0)
            //   Latch instruction from memory  (IRWrite=1)
            //   ALU: OldPC + 4  →  ALUResult
            //   Result = ALUResult  (ResultSrc=10, bypass ALUOut)
            //   PC ← Result = OldPC+4  (PCUpdate=1)
            // ------------------------------------------------
            S0: begin
                IRWrite   = 1;
                AdrSrc    = 0;
                ALUSrcA   = 0;      // OldPC
                ALUSrcB   = 2'b10;  // 4
                ALUOp     = 2'b00;  // ADD
                ResultSrc = 2'b10;  // ALUResult bypass
                PCUpdate  = 1;
            end

            // ------------------------------------------------
            // S1 DECODE
            //   Pre-compute OldPC + ImmExt for BEQ / JAL target
            //   Result stored in ALUOut (used two states later)
            // ------------------------------------------------
            S1: begin
                ALUSrcA = 0;        // OldPC
                ALUSrcB = 2'b01;    // ImmExt
                ALUOp   = 2'b00;    // ADD
                case (op)
                    7'b0000011: ImmSrc = 2'b00; // lw  I-type
                    7'b0100011: ImmSrc = 2'b01; // sw  S-type
                    7'b0010011: ImmSrc = 2'b00; // addi I-type
                    7'b1101111: ImmSrc = 2'b11; // jal J-type
                    7'b1100011: ImmSrc = 2'b10; // beq B-type
                    default:    ImmSrc = 2'b00;
                endcase
            end

            // ------------------------------------------------
            // S2 MEM_ADR  –  address = rs1 + imm
            // ------------------------------------------------
            S2: begin
                ALUSrcA = 1;                    // A (rs1)
                ALUSrcB = 2'b01;                // ImmExt
                ALUOp   = 2'b00;                // ADD
                ImmSrc  = op[5] ? 2'b01 : 2'b00; // S or I
            end

            // ------------------------------------------------
            // S3 MEM_READ  –  hold memory address, let RAM read
            // ------------------------------------------------
            S3: begin
                AdrSrc = 1;   // Adr = ALUOut (memory address)
            end

            // ------------------------------------------------
            // S4 MEM_WB  –  rd ← Data (from memory)
            // ------------------------------------------------
            S4: begin
                ResultSrc = 2'b01;  // Data register
                RegWrite  = 1;
            end

            // ------------------------------------------------
            // S5 MEM_WRITE  –  RAM[ALUOut] ← WriteData
            // ------------------------------------------------
            S5: begin
                AdrSrc   = 1;   // Adr = ALUOut
                MemWrite = 1;
            end

            // ------------------------------------------------
            // S6 EXECUTE_R  –  R-type ALU operation
            // ------------------------------------------------
            S6: begin
                ALUSrcA = 1;        // A  (rs1)
                ALUSrcB = 2'b00;    // RD2 (rs2)
                ALUOp   = 2'b10;    // decode funct3/7
            end

            // ------------------------------------------------
            // S7 ALU_WB  –  rd ← ALUOut
            // ------------------------------------------------
            S7: begin
                ResultSrc = 2'b00;  // ALUOut
                RegWrite  = 1;
            end

            // ------------------------------------------------
            // S8 EXECUTE_I  –  I-type ALU (addi, slti, ori, andi)
            // ------------------------------------------------
            S8: begin
                ALUSrcA = 1;        // A (rs1)
                ALUSrcB = 2'b01;    // ImmExt
                ALUOp   = 2'b10;    // decode funct3
                ImmSrc  = 2'b00;    // I-type
            end

            // ------------------------------------------------
            // S9 JAL
            //   ALUOut (from S1) = OldPC + ImmJ = jump target
            //   JAL=1: PC ← ALUOut  (bypass Result mux for PC)
            //   ALU recomputes OldPC+4 for return address:
            //     ALUSrcA=0 (OldPC), ALUSrcB=10 (4) → ALUResult = OldPC+4
            //   ResultSrc=10: Result = ALUResult = OldPC+4
            //   RegWrite=1: rd ← OldPC+4  (return address ✓)
            // ------------------------------------------------
            S9: begin
                ALUSrcA   = 0;      // OldPC
                ALUSrcB   = 2'b10;  // 4
                ALUOp     = 2'b00;  // ADD → OldPC+4
                ResultSrc = 2'b10;  // ALUResult = OldPC+4
                RegWrite  = 1;      // rd ← OldPC+4
                JAL       = 1;      // PC ← ALUOut (jump target from S1)
            end

            // ------------------------------------------------
            // S10 BEQ
            //   ALU: A - RD2  →  Zero flag
            //   If Zero: PC ← ALUOut (= branch target from S1)
            //   Branch=1, PCWrite = Branch & Zero
            //   ResultSrc=00: Result = ALUOut = branch target
            // ------------------------------------------------
            S10: begin
                ALUSrcA   = 1;      // A  (rs1)
                ALUSrcB   = 2'b00;  // RD2 (rs2)
                ALUOp     = 2'b01;  // SUB
                ResultSrc = 2'b00;  // ALUOut = branch target
                Branch    = 1;
            end

            default: begin end
        endcase
    end
endmodule

// ============================================================
// ALU DECODER
// ALUOp encoding: 00=ADD, 01=SUB, 10=decode funct3/7
// ALUControl:     000=ADD, 001=SUB, 010=AND, 011=OR, 101=SLT
// ============================================================
module aludec (
    input  logic       opb5,
    input  logic [2:0] funct3,
    input  logic       funct7b5,
    input  logic [1:0] ALUOp,
    output logic [2:0] ALUControl
);
    logic RtypeSub;
    assign RtypeSub = funct7b5 & opb5;   // sub only for R-type (op=0110011)

    always_comb
        case (ALUOp)
            2'b00:  ALUControl = 3'b000;   // force ADD
            2'b01:  ALUControl = 3'b001;   // force SUB
            default:                       // decode from funct3 / funct7
                case (funct3)
                    3'b000: ALUControl = RtypeSub ? 3'b001 : 3'b000; // sub / add
                    3'b010: ALUControl = 3'b101;  // slt
                    3'b110: ALUControl = 3'b011;  // or
                    3'b111: ALUControl = 3'b010;  // and
                    default: ALUControl = 3'b000;
                endcase
        endcase
endmodule

// ============================================================
// DATAPATH
// ============================================================
module datapath (
    input  logic        clk, reset,
    input  logic [31:0] ReadData,
    // Control inputs
    input  logic        PCWrite, AdrSrc, IRWrite, RegWrite, ALUSrcA,
    input  logic [1:0]  ALUSrcB, ResultSrc, ImmSrc,
    input  logic [2:0]  ALUControl,
    input  logic        JAL,             // 1 → PC ← ALUOut (overrides Result mux)
    // Status outputs
    output logic        Zero,
    output logic [6:0]  op,
    output logic [2:0]  funct3,
    output logic        funct7b5,
    // Memory interface
    output logic [31:0] Adr, WriteData
);
    logic [31:0] PC, OldPC;
    logic [31:0] Instr, Data;
    logic [31:0] A, RD2reg;
    logic [31:0] ALUResult, ALUOut;
    logic [31:0] ImmExt;
    logic [31:0] SrcA, SrcB, Result;
    logic [31:0] RD1, RD2;

    // ---- PC register ----
    // Priority: reset > JAL override > PCWrite
    always_ff @(posedge clk or posedge reset)
        if (reset)        PC <= 32'h0;
        else if (JAL)     PC <= ALUOut;    // jump target (from S1 Decode)
        else if (PCWrite) PC <= Result;    // normal PC update

    // ---- OldPC register  (captured when instruction is latched) ----
    always_ff @(posedge clk)
        if (IRWrite) OldPC <= PC;

    // ---- Instruction register  (IR) ----
    always_ff @(posedge clk)
        if (IRWrite) Instr <= ReadData;

    // ---- Data register  (latches memory read data) ----
    always_ff @(posedge clk)
        Data <= ReadData;

    // ---- Instruction field decode ----
    assign op       = Instr[6:0];
    assign funct3   = Instr[14:12];
    assign funct7b5 = Instr[30];

    // ---- Immediate extender ----
    extend ext (
        .instr  (Instr[31:7]),
        .immsrc (ImmSrc),
        .immext (ImmExt)
    );

    // ---- Register file ----
    regfile rf (
        .clk (clk), .we3 (RegWrite),
        .a1  (Instr[19:15]),
        .a2  (Instr[24:20]),
        .a3  (Instr[11:7]),
        .wd3 (Result),
        .rd1 (RD1), .rd2 (RD2)
    );

    // ---- A and B pipeline registers ----
    always_ff @(posedge clk) begin
        A      <= RD1;
        RD2reg <= RD2;
    end

    assign WriteData = RD2reg;  // store data for sw

    // ---- SrcA mux  ----
    // When IRWrite=1 (S0 Fetch): SrcA = PC  (compute PC+4)
    // When ALUSrcA=0 (S1 Decode): SrcA = OldPC  (compute OldPC+ImmExt for branches)
    // When ALUSrcA=1 (Execute):   SrcA = A   (register operand)
    assign SrcA = IRWrite ? PC : (ALUSrcA ? A : OldPC);

    // ---- SrcB mux  (00=RD2, 01=ImmExt, 10=4, 11=0) ----
    always_comb
        case (ALUSrcB)
            2'b00: SrcB = RD2reg;
            2'b01: SrcB = ImmExt;
            2'b10: SrcB = 32'd4;
            2'b11: SrcB = 32'd0;
        endcase

    // ---- ALU ----
    alu alu_inst (
        .a          (SrcA),
        .b          (SrcB),
        .alucontrol (ALUControl),
        .result     (ALUResult),
        .zero       (Zero)
    );

    // ---- ALUOut pipeline register ----
    always_ff @(posedge clk)
        ALUOut <= ALUResult;

    // ---- Result mux  (00=ALUOut, 01=Data, 10=ALUResult bypass) ----
    always_comb
        case (ResultSrc)
            2'b00: Result = ALUOut;
            2'b01: Result = Data;
            2'b10: Result = ALUResult;
            2'b11: Result = 32'd0;      // unused
        endcase

    // ---- Address mux  (0=PC, 1=ALUOut) ----
    assign Adr = AdrSrc ? ALUOut : PC;

endmodule

// ============================================================
// REGISTER FILE   (x0 hardwired to 0)
// ============================================================
module regfile (
    input  logic        clk, we3,
    input  logic [4:0]  a1, a2, a3,
    input  logic [31:0] wd3,
    output logic [31:0] rd1, rd2
);
    logic [31:0] rf [31:0];

    always_ff @(posedge clk)
        if (we3 && (a3 != 5'b0))
            rf[a3] <= wd3;

    assign rd1 = (a1 != 5'b0) ? rf[a1] : 32'b0;
    assign rd2 = (a2 != 5'b0) ? rf[a2] : 32'b0;
endmodule

// ============================================================
// ALU
// alucontrol encoding:
//   000 → ADD (a + b)
//   001 → SUB (a − b)
//   010 → AND
//   011 → OR
//   101 → SLT signed  (1 if a < b, 0 otherwise)
// ============================================================
module alu (
    input  logic [31:0] a, b,
    input  logic [2:0]  alucontrol,
    output logic [31:0] result,
    output logic        zero
);
    logic [31:0] condinvb;
    logic [32:0] sum;           // 33-bit to capture carry/borrow

    assign condinvb = alucontrol[0] ? ~b : b;
    assign sum      = {1'b0, a} + {1'b0, condinvb} + {32'b0, alucontrol[0]};

    logic slt_result;
    // Signed SLT: a < b (signed) iff result_sign XOR overflow
    // overflow = (a and b have different signs) AND (result sign differs from a sign)
    // = (a[31] ^ b[31]) & (a[31] ^ sum[31])
    logic overflow, neg;
    assign neg      = sum[31];                                // sign of (a - b)
    assign overflow = (a[31] ^ b[31]) & (a[31] ^ sum[31]);  // signed overflow
    assign slt_result = neg ^ overflow;                       // true if a < b signed

    always_comb
        case (alucontrol)
            3'b000: result = sum[31:0];             // ADD
            3'b001: result = sum[31:0];             // SUB
            3'b010: result = a & b;                 // AND
            3'b011: result = a | b;                 // OR
            3'b101: result = {31'b0, slt_result};   // SLT (signed)
            default: result = 32'bx;
        endcase

    assign zero = (result == 32'b0);
endmodule

// ============================================================
// IMMEDIATE EXTENDER
// immsrc encoding: 00=I-type, 01=S-type, 10=B-type, 11=J-type
// ============================================================
module extend (
    input  logic [31:7] instr,
    input  logic [1:0]  immsrc,
    output logic [31:0] immext
);
    // Extract sign bit and fields as wires (avoids iverilog constant-select limitation)
    logic        sign;
    logic [11:0] i_imm;
    logic [11:0] s_imm;
    logic [12:0] b_imm;
    logic [20:0] j_imm;

    assign sign  = instr[31];
    assign i_imm = instr[31:20];
    assign s_imm = {instr[31:25], instr[11:7]};
    assign b_imm = {instr[31], instr[7], instr[30:25], instr[11:8], 1'b0};
    assign j_imm = {instr[31], instr[19:12], instr[20], instr[30:21], 1'b0};

    always_comb
        case (immsrc)
            2'b00: immext = {{20{sign}}, i_imm};
            2'b01: immext = {{20{sign}}, s_imm};
            2'b10: immext = {{19{sign}}, b_imm};
            2'b11: immext = {{11{sign}}, j_imm};
        endcase
endmodule
