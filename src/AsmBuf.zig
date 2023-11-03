const std = @import("std");
const builtin = @import("builtin");

pub const AsmBuf = extern struct {
    code: [std.mem.page_size - @sizeOf(usize)]u8,
    len: usize,

    pub fn create() !*align(std.mem.page_size) AsmBuf {
        const buf: *align(std.mem.page_size) AsmBuf = switch (builtin.os.tag) {
            .linux, .macos => @ptrCast(try std.os.mmap(
                null,
                @sizeOf(AsmBuf),
                std.os.PROT.READ | std.os.PROT.WRITE,
                std.os.MAP.ANONYMOUS | std.os.MAP.PRIVATE,
                -1,
                0,
            )),
            .windows => @ptrCast(@alignCast(try std.os.windows.VirtualAlloc(
                null,
                @sizeOf(AsmBuf),
                std.os.windows.MEM_RESERVE | std.os.windows.MEM_COMMIT,
                std.os.windows.PAGE_READWRITE,
            ))),
            else => @compileError("only linux, macos, and windows are supported"),
        };
        @memset(&buf.code, undefined);
        buf.len = 0;
        return buf;
    }

    pub fn addInstruction(self: *AsmBuf, size: usize, inst: u64) error{Overflow}!void {
        if (self.len + size > self.code.len) {
            return error.Overflow;
        }

        const all_bytes: [8]u8 = @bitCast(std.mem.nativeToBig(u64, inst));
        const bytes_to_copy = all_bytes[8 - size .. 8];
        @memcpy(self.code[self.len..][0..size], bytes_to_copy);
        self.len += size;
    }

    pub fn addImmediate(self: *AsmBuf, value: []const u8) error{Overflow}!void {
        if (self.len + value.len > self.code.len) {
            return error.Overflow;
        }

        @memcpy(self.code[self.len..][0..value.len], value);
        self.len += value.len;
    }

    pub fn finalize(self: *align(std.mem.page_size) AsmBuf, comptime T: type) !T {
        try std.os.mprotect(std.mem.asBytes(self), std.os.PROT.READ | std.os.PROT.EXEC);
        return @ptrCast(&self.code);
    }

    pub fn destroy(self: *align(std.mem.page_size) AsmBuf) void {
        switch (builtin.os.tag) {
            .linux, .macos => std.os.munmap(std.mem.asBytes(self)),
            .windows => std.os.windows.VirtualFree(self, 0, std.os.windows.MEM_RELEASE),
            else => @compileError("only linux, macos, and windows are supported"),
        }
    }
};
