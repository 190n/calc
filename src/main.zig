const std = @import("std");

const Op = union(enum) {
    add: void,
    sub: void,
    mul: void,
    div: void,
    constant: f64,
};

const Program = struct {
    code: std.BoundedArray(Op, 256) = .{},
    peak_stack_size: u16 = 0,
    args_required: u16 = 0,

    const ParseError = std.fmt.ParseFloatError || error{ InvalidOperator, TooLong };

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
                        p.args_required += 1;
                    }

                    current_stack_size -= 1;
                }
            }
        }

        return p;
    }
};

const AsmBuf = extern struct {
    code: [std.mem.page_size - @sizeOf(usize)]u8,
    len: usize,

    pub fn create() !*align(std.mem.page_size) AsmBuf {
        const buf: *align(std.mem.page_size) AsmBuf = @ptrCast(try std.os.mmap(
            null,
            @sizeOf(AsmBuf),
            std.os.PROT.READ | std.os.PROT.WRITE,
            std.os.MAP.ANONYMOUS | std.os.MAP.PRIVATE,
            -1,
            0,
        ));
        @memset(&buf.code, undefined);
        buf.len = 0;
        return buf;
    }

    pub fn addInstruction(self: *AsmBuf, size: usize, inst: u64) void {
        const all_bytes: [8]u8 = @bitCast(std.mem.nativeToBig(u64, inst));
        const bytes_to_copy = all_bytes[8 - size .. 8];
        @memcpy(self.code[self.len..][0..size], bytes_to_copy);
        self.len += size;
    }

    pub fn addImmediate(self: *AsmBuf, value: []const u8) void {
        @memcpy(self.code[self.len..][0..value.len], value);
        self.len += value.len;
    }

    pub fn finalize(self: *align(std.mem.page_size) AsmBuf) !void {
        return try std.os.mprotect(std.mem.asBytes(self), std.os.PROT.READ | std.os.PROT.EXEC);
    }

    pub fn destroy(self: *align(std.mem.page_size) AsmBuf) void {
        std.os.munmap(std.mem.asBytes(self));
    }
};

pub fn main() !void {
    const argv = try std.process.argsAlloc(std.heap.page_allocator);
    defer std.heap.page_allocator.free(argv);

    var buf = try AsmBuf.create();
    defer buf.destroy();

    buf.addInstruction(2, 0x48b8);
    buf.addImmediate(std.mem.asBytes(&std.mem.nativeToLittle(u64, @bitCast(@as(f64, 1.0)))));
    buf.addInstruction(5, 0x66480f6ec8);
    buf.addInstruction(4, 0xf20f58c1);
    buf.addInstruction(1, 0xc3);

    try buf.finalize();

    const func: *const fn (f64) callconv(.C) f64 = @ptrCast(&buf.code);
    std.debug.print("{}\n", .{func(6.9)});

    if (argv.len != 2) {
        return error.WrongNumberOfArguments;
    }

    const program = try Program.parse(argv[1]);

    std.debug.print("{} {}\n", .{ program.peak_stack_size, program.args_required });

    var input = std.io.getStdIn().reader();
    var line_buf: [1024]u8 = undefined;
    while (try input.readUntilDelimiterOrEof(&line_buf, '\n')) |line| {
        var it = std.mem.tokenizeScalar(u8, line, ' ');
        _ = it;
    }
}
