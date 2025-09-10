module SingleCycleMIPS (
    input clk,
    input reset
);

    // ========== Instruction Fetch Stage ==========
    // ========== WIRE DECLARATIONS ==========
    wire [31:0] PC;
    wire [31:0] PCPlus4;
    wire [31:0] Instruction;
    wire InvalidAddressError;
    wire [31:0] NextPC;

    wire [4:0] WriteReg;
    wire [31:0] ReadData1, ReadData2;
    wire [31:0] WriteData;

    wire [31:0] ALUResult;
    wire Zero;
    wire [31:0] SignExtended;
    wire [31:0] ALUInput2;
    wire [2:0] ALUControl;

    wire [31:0] ReadData;

    wire [31:0] ShiftedOffset;
    wire [31:0] BranchTarget;
    wire [31:0] JumpTarget;
    wire BranchTaken;

    wire [31:0] BranchMuxOut;

    wire RegWrite, ALUSrc, MemRead, MemWrite, MemtoReg, Branch, Jump, RegDest;
    wire [1:0] ALUOp;




    
    // PC Register
    reg [31:0] PC_reg;
    always @(posedge clk or posedge reset) begin
        if (reset) PC_reg <= 32'h00000000;
        else PC_reg <= NextPC;
    end
    assign PC = PC_reg;
    
    // Instruction Memory
    InstructionMemory IM (
        .clk(clk),
        .Address(PC),
        .Instruction(Instruction),
        .InvalidAddressError(InvalidAddressError)
    );
    
    // PC+4 Adder
    Adder PCAdder (
        .A(PC),
        .B(32'd4),
        .Result(PCPlus4)
    );
    
    // ========== Register File Stage ==========

    
    // RegDst MUX (5-bit)
    MUX_5bit_2to1 RegDstMux (
        .Enable(1'b1),
        .In0(Instruction[20:16]), // rt
        .In1(Instruction[15:11]), // rd
        .Sel(RegDest),
        .Out(WriteReg)
    );
    
    RegisterFile RF (
        .Clk(clk),
        .RegWrite(RegWrite),
        .ReadReg1(Instruction[25:21]), // rs
        .ReadReg2(Instruction[20:16]), // rt
        .WriteReg(WriteReg),
        .WriteData(WriteData),
        .ReadData1(ReadData1),
        .ReadData2(ReadData2)
    );
    
    // ========== ALU Stage ==========

    
    // Sign Extension
    SignExtend SE (
        .Input16(Instruction[15:0]),
        .Output32(SignExtended)
    );
    
    // ALUSrc MUX (32-bit)
    MUX_32bit_2to1 ALUSrcMux (
        .Enable(1'b1),
        .In0(ReadData2),
        .In1(SignExtended),
        .Sel(ALUSrc),
        .Out(ALUInput2)
    );
    
    // ALU Decoder
    ALUDecoder ALUDec (
        .ALUOp(ALUOp),
        .Funct(Instruction[5:0]),
        .ALUControl(ALUControl)
    );
    
    // ALU
    ALU ALU (
        .A(ReadData1),
        .B(ALUInput2),
        .ALUOp(ALUControl),
        .Result(ALUResult),
        .Zero(Zero)
    );
    
    // ========== Data Memory Stage ==========

    
    DataMemory DM (
        .Clk(clk),
        .MemRead(MemRead),
        .MemWrite(MemWrite),
        .Address(ALUResult),
        .WriteData(ReadData2),
        .ReadData(ReadData)
    );
    
    // MemtoReg MUX (32-bit)
    MUX_32bit_2to1 MemtoRegMux (
        .Enable(1'b1),
        .In0(ALUResult),
        .In1(ReadData),
        .Sel(MemtoReg),
        .Out(WriteData)
    );
    
    // ========== Branch/Jump Logic ==========

    
    // Shift Left 2 for branch offset
    ShiftLeft2 SL2 (
        .Input(SignExtended),
        .Output(ShiftedOffset)
    );
    
    // Branch Target Adder
    Adder BranchAdder (
        .A(PCPlus4),
        .B(ShiftedOffset),
        .Result(BranchTarget)
    );
    
    // Jump Target Calculation
    assign JumpTarget = {PCPlus4[31:28], Instruction[25:0], 2'b00};
    
    // Branch condition
    assign BranchTaken = Branch & Zero;


    MUX_32bit_2to1 BranchMux (
        .Enable(1'b1),
        .In0(PCPlus4),
        .In1(BranchTarget),
        .Sel(BranchTaken),
        .Out(BranchMuxOut)
    );

    MUX_32bit_2to1 PCSrcMux (
        .Enable(1'b1),
        .In0(BranchMuxOut),
        .In1(JumpTarget),
        .Sel(Jump),
        .Out(NextPC)
    );
    
    // ========== Control Unit ==========

    
    ControlUnit CU (
        .Opcode(Instruction[31:26]),
        .RegWrite(RegWrite),
        .ALUSrc(ALUSrc),
        .MemRead(MemRead),
        .MemWrite(MemWrite),
        .MemtoReg(MemtoReg),
        .Branch(Branch),
        .Jump(Jump),
        .RegDest(RegDest),
        .ALUOp(ALUOp)
    );
    
endmodule








module InstructionMemory (
    input wire clk,                    // Clock input
    input wire [31:0] Address,         // Program Counter (PC) input
    output reg [31:0] Instruction,     // Output instruction
    output reg InvalidAddressError     // High if address is out-of-bounds
);

    // 1024 x 32-bit memory array (1KB total)
    reg [31:0] Mem[0:1023];

    // Synchronous read with bounds checking
    always @(posedge clk) begin
        if (Address[31:2] > 1023) begin
            Instruction <= 32'h00000000;     // Return NOP
            InvalidAddressError <= 1'b1;     // Raise error
        end else begin
            Instruction <= Mem[Address[31:2]]; // Word-aligned read
            InvalidAddressError <= 1'b0;
        end
    end

endmodule



module RegisterFile (
    input Clk,                     // Clock signal
    input RegWrite,                // Write enable (1 = write to register)
    input [4:0] ReadReg1,          // Address of first register to read (rs1)
    input [4:0] ReadReg2,          // Address of second register to read (rs2)
    input [4:0] WriteReg,          // Address of register to write (rd)
    input [31:0] WriteData,        // Data to be written
    output reg [31:0] ReadData1,   // Output of first register read
    output reg [31:0] ReadData2    // Output of second register read
);

    // 32 registers, each 32 bits wide
    reg [31:0] Registers [0:31];

    // Initialize all registers to 0 (except x0, which is always 0)
    integer i;
    initial begin
        for (i = 0; i < 32; i = i + 1)
            Registers[i] = 32'b0;
    end

    // Combinational read (asynchronous)
    always @(*) begin
        ReadData1 = (ReadReg1 == 0) ? 32'b0 : Registers[ReadReg1]; // x0 is always 0
        ReadData2 = (ReadReg2 == 0) ? 32'b0 : Registers[ReadReg2]; // x0 is always 0
    end

    // Synchronous write (on rising clock edge)
    always @(posedge Clk) begin
        if (RegWrite && (WriteReg != 0)) // x0 cannot be written
            Registers[WriteReg] <= WriteData;
    end

endmodule

module ALU (
    input [31:0] A,         // First operand
    input [31:0] B,         // Second operand
    input [2:0] ALUOp,      // Operation selector
    output reg [31:0] Result, // ALU result
    output Zero             // Zero flag (1 if Result == 0)
);

    // Compute Zero flag (combinational)
    assign Zero = (Result == 32'b0);

    // ALU Operations
    always @(*) begin
        case (ALUOp)
            3'b000: Result = A + B;      // ADD
            3'b001: Result = A - B;      // SUB
            3'b010: Result = A & B;      // AND
            3'b011: Result = A | B;      // OR
            3'b100: Result = ~(A | B);   // NOR
            3'b101: Result = (A < B) ? 32'd1 : 32'd0; // SLT
            default: Result = 32'b0;     // Default case (optional)
        endcase
    end

endmodule


module DataMemory (
    input        Clk,
    input        MemRead,      
    input        MemWrite,
    input  [31:0] Address,     // Word-aligned (LSB 2 bits ignored)
    input  [31:0] WriteData,   // Data to store (for sw)
    output reg [31:0] ReadData // Data read (for lw)
);

    // 1KB Memory (256 words x 32-bit)
    reg [31:0] Mem [0:255];

    // Combinational read: only drive ReadData when MemRead is asserted
    always @(*) begin
        if (MemRead)
            ReadData = Mem[Address[31:2]]; // Word addressing (ignore last 2 bits)
        else
            ReadData = 32'b0;
    end

    // Synchronous write (only on sw)
    always @(posedge Clk) begin
        if (MemWrite)
            Mem[Address[31:2]] <= WriteData;
    end
endmodule



module ControlUnit (
    input [5:0] Opcode,         // Instruction opcode (bits 31:26)
    output reg RegWrite,        // Register write enable
    output reg ALUSrc,          // ALU source (0=reg, 1=imm)
    output reg MemRead,         // Memory read enable
    output reg MemWrite,        // Memory write enable
    output reg MemtoReg,        // Result select (0=ALU, 1=Mem)
    output reg Branch,          // Branch enable
    output reg Jump,            // Jump enable
    output reg RegDest,         // Destination register select (0=rt, 1=rd)
    output reg [1:0] ALUOp      // ALU operation (refined by ALU decoder)
);

    always @(*) begin
        // Default values (safe state)
        RegWrite  = 0;
        ALUSrc    = 0;
        MemRead   = 0;
        MemWrite  = 0;
        MemtoReg  = 0;
        Branch    = 0;
        Jump      = 0;
        RegDest   = 0;
        ALUOp     = 2'b00;

        case (Opcode)
            6'b000000: begin // R-type (ADD, SUB, AND, OR, SLT, etc.)
                RegWrite = 1;
                RegDest  = 1;     // Destination is rd
                ALUSrc   = 0;     // Second ALU operand is register
                ALUOp    = 2'b10; // ALU decoder uses funct field
            end

            6'b100011: begin // LW (Load Word)
                RegWrite  = 1;
                RegDest   = 0;    // Destination is rt
                ALUSrc    = 1;    // Use immediate as second ALU operand
                MemRead   = 1;
                MemtoReg  = 1;    // Write memory data to register
                ALUOp     = 2'b00; // ALU does base + offset
            end

            6'b101011: begin // SW (Store Word)
                ALUSrc   = 1;
                MemWrite = 1;
                ALUOp    = 2'b00; // ALU does base + offset
            end

            6'b000100: begin // BEQ (Branch if Equal)
                Branch = 1;
                ALUOp  = 2'b01; // ALU does subtraction for comparison
            end

            6'b001000: begin // ADDI (Add Immediate)
                RegWrite = 1;
                RegDest  = 0;    // Destination is rt
                ALUSrc   = 1;
                ALUOp    = 2'b00;
            end

            6'b000010: begin // JUMP (J)
                Jump = 1;
            end

            default: begin
                // Do nothing, all control signals remain zero
            end
        endcase
    end

endmodule



module ALUDecoder (
    input [1:0] ALUOp,      // From Control Unit
    input [5:0] Funct,      // funct field (bits 5:0)
    output reg [2:0] ALUControl // Final ALU control
);

    always @(*) begin
        casex ({ALUOp, Funct})
            8'b10_100000: ALUControl = 3'b000; // ADD
            8'b10_100010: ALUControl = 3'b001; // SUB
            8'b10_100100: ALUControl = 3'b010; // AND
            8'b10_100101: ALUControl = 3'b011; // OR
            8'b10_101010: ALUControl = 3'b101; // SLT
            default: ALUControl = 3'b000; // Default to ADD
        endcase
    end

endmodule

module MUX_5bit_2to1 (
    input Enable,        // 1-bit enable signal
    input [4:0] In0,    // First 5-bit input
    input [4:0] In1,    // Second 5-bit input
    input Sel,          // Selection line (0=In0, 1=In1)
    output reg [4:0] Out // 5-bit output
);

    always @(*) begin
        if (!Enable) 
            Out = 5'b00000; // Disabled (outputs zeros)
        else 
            Out = Sel ? In1 : In0; // Enabled (selects In1 or In0)
    end

endmodule

module MUX_32bit_2to1 (
    input Enable,        // 1-bit enable signal
    input [31:0] In0,    // First 5-bit input
    input [31:0] In1,    // Second 5-bit input
    input Sel,          // Selection line (0=In0, 1=In1)
    output reg [31:0] Out // 5-bit output
);

    always @(*) begin
        if (!Enable) 
            Out = 32'b00000000000000000000000000000000; // Disabled (outputs zeros)
        else 
            Out = Sel ? In1 : In0; // Enabled (selects In1 or In0)
    end

endmodule

module SignExtend (
    input [15:0] Input16,   // 16-bit input
    output reg [31:0] Output32  // 32-bit sign-extended output
);

    always @(*) begin
        // Replicate the sign bit (MSB) for upper 16 bits
        Output32 = {{16{Input16[15]}}, Input16};
    end

endmodule

module ShiftLeft2 (
    input [31:0] Input,     // 32-bit input
    output [31:0] Output    // 32-bit shifted output
);

    assign Output = {Input[29:0], 2'b00};  // Shift left by 2, fill LSBs with 0

endmodule


module Adder (
    input [31:0] A,        // First operand
    input [31:0] B,        // Second operand
    output [31:0] Result   // A + B
);
    assign Result = A + B;  // Pure combinational logic
endmodule


























