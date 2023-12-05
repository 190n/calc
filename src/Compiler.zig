const std = @import("std");
const builtin = @import("builtin");

const Program = @import("./Program.zig");
const Assembler = @import("./Assembler.zig");

const Compiler = @This();

pub const Constant = extern union {
    float: f64,
    int: u64,
};

constants: std.ArrayList(Constant),
stack_top: u16,

pub fn initWithConstants(allocator: std.mem.Allocator, program: Program) !Compiler {
    var compiler = Compiler{
        .constants = std.ArrayList(Constant).init(allocator),
        .stack_top = program.num_args,
    };

    for (program.code.items) |inst| {
        switch (inst) {
            .constant => |c| {
                if (std.mem.indexOfScalar(f64, @ptrCast(compiler.constants.items), c) == null) {
                    try compiler.constants.append(.{ .float = c });
                }
            },
            .call_unary => |f| {
                if (std.mem.indexOfScalar(u64, @ptrCast(compiler.constants.items), @intFromPtr(f)) == null) {
                    try compiler.constants.append(.{ .int = @intFromPtr(f) });
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
            try assembler.assemble(.{ .load = .{ .dst = .a, .src = .{ .base = .vm_stack, .offset = 8 * (self.stack_top - 2) } } });
            try assembler.assemble(.{ .load = .{ .dst = .b, .src = .{ .base = .vm_stack, .offset = 8 * (self.stack_top - 1) } } });
            try assembler.assemble(.{ .sub_float = .{ .dst = .a, .src1 = .a, .src2 = .b } });
            try assembler.assemble(.{ .store = .{ .dst = .{ .base = .vm_stack, .offset = 8 * (self.stack_top - 2) }, .src = .a } });
            self.stack_top -= 1;
        },
        .mul => {
            try assembler.assemble(.{ .load = .{ .dst = .a, .src = .{ .base = .vm_stack, .offset = 8 * (self.stack_top - 2) } } });
            try assembler.assemble(.{ .load = .{ .dst = .b, .src = .{ .base = .vm_stack, .offset = 8 * (self.stack_top - 1) } } });
            try assembler.assemble(.{ .mul_float = .{ .dst = .a, .src1 = .a, .src2 = .b } });
            try assembler.assemble(.{ .store = .{ .dst = .{ .base = .vm_stack, .offset = 8 * (self.stack_top - 2) }, .src = .a } });
            self.stack_top -= 1;
        },
        .div => {
            try assembler.assemble(.{ .load = .{ .dst = .a, .src = .{ .base = .vm_stack, .offset = 8 * (self.stack_top - 2) } } });
            try assembler.assemble(.{ .load = .{ .dst = .b, .src = .{ .base = .vm_stack, .offset = 8 * (self.stack_top - 1) } } });
            try assembler.assemble(.{ .div_float = .{ .dst = .a, .src1 = .a, .src2 = .b } });
            try assembler.assemble(.{ .store = .{ .dst = .{ .base = .vm_stack, .offset = 8 * (self.stack_top - 2) }, .src = .a } });
            self.stack_top -= 1;
        },

        .constant => |c| {
            const index = std.mem.indexOfScalar(f64, @ptrCast(self.constants.items), c).?;

            try assembler.assemble(.{ .load = .{ .dst = .a, .src = .{ .base = .constants, .offset = @intCast(8 * index) } } });
            try assembler.assemble(.{ .store = .{ .dst = .{ .base = .vm_stack, .offset = 8 * self.stack_top }, .src = .a } });

            self.stack_top += 1;
        },

        .call_unary => |f| {
            const index = std.mem.indexOfScalar(u64, @ptrCast(self.constants.items), @intFromPtr(f)).?;

            try assembler.assemble(.{ .load = .{ .dst = .a, .src = .{ .base = .vm_stack, .offset = 8 * (self.stack_top - 1) } } });
            try assembler.assemble(.{ .call_unary = .{ .index = @intCast(index), .dst = .a, .src = .a } });
            try assembler.assemble(.{ .store = .{ .dst = .{ .base = .vm_stack, .offset = 8 * (self.stack_top - 1) }, .src = .a } });
        },

        .dup => {
            try assembler.assemble(.{ .load = .{ .dst = .a, .src = .{ .base = .vm_stack, .offset = 8 * (self.stack_top - 1) } } });
            try assembler.assemble(.{ .store = .{ .dst = .{ .base = .vm_stack, .offset = 8 * self.stack_top }, .src = .a } });
            self.stack_top += 1;
        },

        .pop => {
            self.stack_top -= 1;
        },
    }
}

pub fn getConstants(self: *Compiler) ![]Constant {
    return self.constants.toOwnedSlice();
}
