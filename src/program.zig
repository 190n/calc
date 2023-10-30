const std = @import("std");

const AsmBuf = @import("./asmbuf.zig").AsmBuf;

const Program = @This();

pub const Op = union(enum) {
    add: void,
    sub: void,
    mul: void,
    div: void,
    constant: f64,
};

code: std.BoundedArray(Op, 256) = .{},
peak_stack_size: u16 = 0,
num_args: u16 = 0,
num_returns: u16 = 0,

pub const ParseError = std.fmt.ParseFloatError || error{ InvalidOperator, TooLong };

pub fn parse(text: []const u8) ParseError!Program {
    var p = Program{};
    var current_stack_size: u16 = 0;

    var it = std.mem.tokenizeScalar(u8, text, ' ');
    while (it.next()) |s| {
        if (std.fmt.parseFloat(f64, s)) |f| {
            p.code.append(.{ .constant = f }) catch return error.TooLong;
            current_stack_size += 1;
            p.peak_stack_size = @max(current_stack_size, p.peak_stack_size);
        } else |_| {
            if (s.len == 1) {
                p.code.append(switch (s[0]) {
                    '+' => .add,
                    '-' => .sub,
                    '*' => .mul,
                    '/' => .div,
                    else => return error.InvalidOperator,
                }) catch return error.TooLong;

                while (current_stack_size < 2) {
                    current_stack_size += 1;
                    p.peak_stack_size += 1;
                    p.num_args += 1;
                }

                current_stack_size -= 1;
            }
        }
    }

    p.num_returns = current_stack_size;
    return p;
}

fn writeOffset(asm_buf: *AsmBuf, register: u8, offset: u16) !void {
    const byte_offset = @sizeOf(f64) * offset;

    if (byte_offset <= 0x7f) {
        try asm_buf.addImmediate(&.{0x40 | register});
        try asm_buf.addImmediate(&.{@truncate(byte_offset)});
    } else {
        try asm_buf.addImmediate(&.{0x80 | register});
        try asm_buf.addImmediate(std.mem.asBytes(&std.mem.nativeToLittle(u32, byte_offset)));
    }
}

pub fn compile(
    self: *const Program,
    asm_buf: *align(std.mem.page_size) AsmBuf,
) ![256]f64 {
    comptime std.debug.assert(@import("builtin").cpu.arch == .x86_64);

    var constants = [_]f64{0.0} ** 256;
    var num_constants: usize = 0;

    for (self.code.slice()) |inst| {
        switch (inst) {
            .constant => |c| {
                if (std.mem.indexOfScalar(f64, constants[0..num_constants], c) == null) {
                    constants[num_constants] = c;
                    num_constants += 1;
                }
            },
            else => {},
        }
    }

    var top: u16 = self.num_args;

    for (self.code.slice()) |inst| {
        switch (inst) {
            .add => {
                // movsd (8*(top-2))(%rdi), %xmm0
                try asm_buf.addInstruction(3, 0xf20f10);
                try writeOffset(asm_buf, 0x7, top - 2);
                // addsd (8*(top-1))(%rdi), %xmm0
                try asm_buf.addInstruction(3, 0xf20f58);
                try writeOffset(asm_buf, 0x7, top - 1);
                // movsd %xmm0, (8*(top-2))(%rdi)
                try asm_buf.addInstruction(3, 0xf20f11);
                try writeOffset(asm_buf, 0x7, top - 2);
                top -= 1;
            },
            .sub => {
                // movsd (8*(top-2))(%rdi), %xmm0
                try asm_buf.addInstruction(3, 0xf20f10);
                try writeOffset(asm_buf, 0x7, top - 2);
                // subsd (8*(top-1))(%rdi), %xmm0
                try asm_buf.addInstruction(3, 0xf20f5c);
                try writeOffset(asm_buf, 0x7, top - 1);
                // movsd %xmm0, (8*(top-2))(%rdi)
                try asm_buf.addInstruction(3, 0xf20f11);
                try writeOffset(asm_buf, 0x7, top - 2);
                top -= 1;
            },
            .mul => {
                // movsd (8*(top-2))(%rdi), %xmm0
                try asm_buf.addInstruction(3, 0xf20f10);
                try writeOffset(asm_buf, 0x7, top - 2);
                // mulsd (8*(top-1))(%rdi), %xmm0
                try asm_buf.addInstruction(3, 0xf20f59);
                try writeOffset(asm_buf, 0x7, top - 1);
                // movsd %xmm0, (8*(top-2))(%rdi)
                try asm_buf.addInstruction(3, 0xf20f11);
                try writeOffset(asm_buf, 0x7, top - 2);
                top -= 1;
            },
            .div => {
                // movsd (8*(top-2))(%rdi), %xmm0
                try asm_buf.addInstruction(3, 0xf20f10);
                try writeOffset(asm_buf, 0x7, top - 2);
                // divsd (8*(top-1))(%rdi), %xmm0
                try asm_buf.addInstruction(3, 0xf20f5e);
                try writeOffset(asm_buf, 0x7, top - 1);
                // movsd %xmm0, (8*(top-2))(%rdi)
                try asm_buf.addInstruction(3, 0xf20f11);
                try writeOffset(asm_buf, 0x7, top - 2);
                top -= 1;
            },

            .constant => |c| {
                const index = std.mem.indexOfScalar(f64, &constants, c).?;
                // movsd (8*index)(%rsi), %xmm0
                try asm_buf.addInstruction(3, 0xf20f10);
                try writeOffset(asm_buf, 0x6, @intCast(index));
                // movsd %xmm0, (8*top)(%rdi)
                try asm_buf.addInstruction(3, 0xf20f11);
                try writeOffset(asm_buf, 0x7, top);
                top += 1;
            },
        }
    }

    // ret
    try asm_buf.addInstruction(1, 0xc3);
    return constants;
}