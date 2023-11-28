const std = @import("std");
const builtin = @import("builtin");

const Assembler = @import("./Assembler.zig");
const Compiler = @import("./Compiler.zig");
const functions = @import("./functions.zig").functions;

const Program = @This();

pub const Operation = union(enum) {
    add: void,
    sub: void,
    mul: void,
    div: void,
    constant: f64,
    call_unary: *const fn (x: f64) callconv(Assembler.calling_convention) f64,
};

code: std.ArrayListUnmanaged(Operation),
peak_stack_size: u16 = 0,
num_args: u16 = 0,
num_returns: u16 = 0,
allocator: std.mem.Allocator,

pub const ParseError = std.fmt.ParseFloatError || std.mem.Allocator.Error || error{ InvalidProgram, TooLong };

pub fn parse(allocator: std.mem.Allocator, texts: [][]const u8, erroneous_part: ?*[]const u8) ParseError!Program {
    var p = Program{
        .allocator = allocator,
        .code = try std.ArrayListUnmanaged(Operation).initCapacity(allocator, 64),
    };
    var current_stack_size: u16 = 0;

    for (texts) |text| {
        var it = std.mem.tokenizeScalar(u8, text, ' ');
        while (it.next()) |s| {
            if (std.fmt.parseFloat(f64, s)) |f| {
                try p.code.append(allocator, .{ .constant = f });
                current_stack_size += 1;
                p.peak_stack_size = @max(current_stack_size, p.peak_stack_size);
            } else |_| {
                if (functions.get(s)) |f| {
                    try p.code.append(allocator, .{ .call_unary = f });
                    while (current_stack_size < 1) {
                        current_stack_size += 1;
                        p.peak_stack_size += 1;
                        p.num_args += 1;
                    }
                } else if (s.len == 1) {
                    try p.code.append(allocator, switch (s[0]) {
                        '+' => .add,
                        '-' => .sub,
                        '*' => .mul,
                        '/' => .div,
                        else => {
                            if (erroneous_part) |e| e.* = s;
                            return error.InvalidProgram;
                        },
                    });

                    while (current_stack_size < 2) {
                        current_stack_size += 1;
                        p.peak_stack_size += 1;
                        p.num_args += 1;
                    }

                    current_stack_size -= 1;
                } else {
                    if (erroneous_part) |e| e.* = s;
                    return error.InvalidProgram;
                }
            }
        }
    }

    p.num_returns = current_stack_size;
    return p;
}

pub fn deinit(self: *Program) void {
    self.code.deinit(self.allocator);
    self.* = undefined;
}

pub fn compile(self: Program, assembler: *Assembler) ![]Compiler.Constant {
    var compiler = try Compiler.initWithConstants(self.allocator, self);

    try assembler.emitPrologue();
    for (self.code.items) |op| {
        try compiler.compileOperation(assembler, op);
    }
    try assembler.emitEpilogue();
    return compiler.getConstants();
}
