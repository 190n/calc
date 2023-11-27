const std = @import("std");
const builtin = @import("builtin");

const Program = @import("./Program.zig");
const Assembler = @import("./Assembler.zig");

const Compiler = @This();

constants: std.ArrayList(f64),
stack_top: u16,

pub fn initWithConstants(allocator: std.mem.Allocator, program: Program) !Compiler {
    var compiler = Compiler{
        .constants = std.ArrayList(f64).init(allocator),
        .stack_top = program.num_args,
    };

    for (program.code.items) |inst| {
        switch (inst) {
            .constant => |c| {
                if (std.mem.indexOfScalar(f64, compiler.constants.items, c) == null) {
                    try compiler.constants.append(c);
                }
            },
            else => {},
        }
    }

    return compiler;
}

pub fn compileOperation(self: *Compiler, assembler: *Assembler, op: Program.Operation) !void {
    switch (op) {
        .add => {
            try assembler.assemble(.{ .load = .{ .dst = .a, .src = .{ .base = .vm_stack, .offset = 8 * (self.stack_top - 2) } } });
            try assembler.assemble(.{ .load = .{ .dst = .b, .src = .{ .base = .vm_stack, .offset = 8 * (self.stack_top - 1) } } });
            try assembler.assemble(.{ .add_float = .{ .dst = .a, .src1 = .a, .src2 = .b } });
            try assembler.assemble(.{ .store = .{ .dst = .{ .base = .vm_stack, .offset = 8 * (self.stack_top - 2) }, .src = .a } });
            self.stack_top -= 1;
        },
        .sub => {
            unreachable;
        },
        .mul => {
            unreachable;
        },
        .div => {
            unreachable;
        },

        .constant => |c| {
            const index = std.mem.indexOfScalar(f64, self.constants.items, c).?;

            try assembler.assemble(.{ .load = .{ .dst = .a, .src = .{ .base = .constants, .offset = @intCast(8 * index) } } });
            try assembler.assemble(.{ .store = .{ .dst = .{ .base = .vm_stack, .offset = 8 * self.stack_top }, .src = .a } });

            // std.log.debug("movsd {}(%rsi), %xmm0", .{8 * index});
            // std.log.debug("movsd %xmm0, {}(%rdi)", .{8 * self.stack_top});
            // // movsd (8*index)(%rsi), %xmm0
            // try asm_buf.addInstruction(3, false, 0xf20f10);
            // try writeOffset(asm_buf, constants, @intCast(index));
            // // movsd %xmm0, (8*top)(%rdi)
            // try asm_buf.addInstruction(3, false, 0xf20f11);
            // try writeOffset(asm_buf, stack, self.stack_top);
            self.stack_top += 1;
        },
    }
}

pub fn getConstants(self: *Compiler) ![]f64 {
    return self.constants.toOwnedSlice();
}
