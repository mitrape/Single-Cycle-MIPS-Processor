`timescale 1ns / 1ps

module SingleCycleMIPS_tb();

    reg clk;
    reg reset;

    SingleCycleMIPS uut (
        .clk(clk),
        .reset(reset)
    );

    // Clock: 10 ns period (100 MHz)
    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end

    // ========= Instruction & Data Memory Initialization =========
    initial begin
        // R-type
        uut.IM.Mem[0]  = 32'h00221820; // ADD  $3, $1, $2     => $3 = 10 + 5 = 15
        uut.IM.Mem[1] = 32'h00222022; // SUB $4, $1, $2       => $4 = 10 - 5 = 5
        uut.IM.Mem[2]  = 32'h00223024; // AND  $6, $1, $2     => $6 = 10 & 5 = 0
        uut.IM.Mem[3]  = 32'h00223825; // OR   $7, $1, $2     => $7 = 10 | 5 = 15
        uut.IM.Mem[4]  = 32'h0022402A; // SLT  $8, $1, $2     => $8 = ($1 > $2) = 0

        // I-type
        uut.IM.Mem[5]  = 32'h2029000A; // ADDI $9, $1, 10     => $9 = 10 + 10 = 20
        uut.IM.Mem[6]  = 32'h8C2A0008; // LW   $10, 8($1)     => $10 = Mem[4] = 0x12345678
        uut.IM.Mem[7]  = 32'hAC2B000C; // SW   $11, 12($1)    => Mem[5] = $11 = 0xABCDEF12
        uut.IM.Mem[8]  = 32'h10220001; // BEQ  $1, $2, +1     => NOT taken ($1 != $2)

        // J-type
        uut.IM.Mem[9]  = 32'h0800000B; // J    0x0000002C     => PC = 0x2C (index 11)

        // Filler (jump target)
        uut.IM.Mem[11] = 32'h00221820; // ADD $3, $1, $2 (harmless NOP-like op)

        // BEQ (taken)
        uut.IM.Mem[10] = 32'h10210001; // BEQ $1, $1, +1 => Should branch to 12
        uut.IM.Mem[12] = 32'h2012002A; // ADDI $18, $0, 42 => $18 = 42
        uut.IM.Mem[13] = 32'h200DDEAD; // ADDI $13, $0, 0xDEAD => Should be skipped
        



        // Data memory
        uut.DM.Mem[4] = 32'h12345678; // for LW
    end

    // ========= Register Initialization =========
    initial begin
        uut.RF.Registers[1]  = 32'd10;          // $1
        uut.RF.Registers[2]  = 32'd5;           // $2
        uut.RF.Registers[11] = 32'hABCDEF12;    // $11, stored via SW
    end

    // ========= Main Test Sequence =========
    initial begin
        $display("=== Starting Full MIPS Instruction Test ===");
        reset = 1;
        #20;
        reset = 0;

        #140; // To allow execution of instruction 14 (NOR)

        $display("\n=== Checking Results ===");

        // ----- R-Type Tests -----
        if (uut.RF.Registers[3] === 32'd15)
            $display("ADD  Test: SUCCESS ($3 = %h)", uut.RF.Registers[3]);
        else
            $display("ADD  Test: ERROR   ($3 = %h, expected 0F)", uut.RF.Registers[3]);

        if ($signed(uut.RF.Registers[4]) === 5)
            $display("SUB  Test: SUCCESS ($4 = %h)", uut.RF.Registers[4]);
        else
            $display("SUB  Test: ERROR   ($4 = %h, expected )", uut.RF.Registers[4]);

        if (uut.RF.Registers[6] === (10 & 5))
            $display("AND  Test: SUCCESS ($6 = %h)", uut.RF.Registers[6]);
        else
            $display("AND  Test: ERROR   ($6 = %h, expected 0)", uut.RF.Registers[6]);

        if (uut.RF.Registers[7] === (10 | 5))
            $display("OR   Test: SUCCESS ($7 = %h)", uut.RF.Registers[7]);
        else
            $display("OR   Test: ERROR   ($7 = %h, expected 0F)", uut.RF.Registers[7]);

        if (uut.RF.Registers[8] === 0)
            $display("SLT  Test: SUCCESS ($8 = %h)", uut.RF.Registers[8]);
        else
            $display("SLT  Test: ERROR   ($8 = %h, expected 0)", uut.RF.Registers[8]);

        // ----- I-Type Tests -----
        if (uut.RF.Registers[9] === 20)
            $display("ADDI Test: SUCCESS ($9 = %h)", uut.RF.Registers[9]);
        else
            $display("ADDI Test: ERROR   ($9 = %h, expected 14)", uut.RF.Registers[9]);

        if (uut.RF.Registers[10] === 32'h12345678)
            $display("LW   Test: SUCCESS ($10 = %h)", uut.RF.Registers[10]);
        else
            $display("LW   Test: ERROR   ($10 = %h, expected 12345678)", uut.RF.Registers[10]);

        if (uut.DM.Mem[5] === 32'hABCDEF12)
            $display("SW   Test: SUCCESS (Mem[5] = %h)", uut.DM.Mem[5]);
        else
            $display("SW   Test: ERROR   (Mem[5] = %h, expected ABCDEF12)", uut.DM.Mem[5]);

        // ----- BEQ Not Taken -----
        if (uut.PC >= 32'h0000002C)
            $display("BEQ (not taken) Test: SUCCESS (continued as expected)");
        else
            $display("BEQ (not taken) Test: ERROR (PC = %h, expected >= 0000002C)", uut.PC);

        // ----- JUMP Test -----
        if (uut.PC >= 32'h0000002C)
            $display("JUMP Test: SUCCESS (PC = %h)", uut.PC);
        else
            $display("JUMP Test: ERROR   (PC = %h, expected >= 0000002C)", uut.PC);

        // ----- BEQ Taken -----
        if (uut.RF.Registers[18] === 32'd42 && uut.RF.Registers[13] === 32'd0)
            $display("BEQ (taken) Test: SUCCESS (Branch taken, $18 = %0d, $13 = %0d)", uut.RF.Registers[18], uut.RF.Registers[13]);
        else begin
            if (uut.RF.Registers[18] !== 32'd42)
                $display("BEQ (taken) Test: ERROR ($18 = %0d, expected 42)", uut.RF.Registers[18]);
            if (uut.RF.Registers[13] !== 32'd0)
                $display("BEQ (taken) Test: ERROR ($13 = %0d, expected 0 — skipped instruction executed?)", uut.RF.Registers[13]);
        end

        $display("\n=== All Tests Complete ===");
        $finish;
    end

    // Real-time Trace
    always @(posedge clk) begin
        $display("PC: %h, Instruction: %h", uut.PC, uut.Instruction);
    end

endmodule
