"""ASCII telemetry append with kernel readback; no shared TextIO encoder buffer."""
import fcntl
import os
import threading


class MetricsIntegrityError(OSError):
    pass


class VerifiedAsciiAppendLog:
    """Keep the existing line protocol and reject any immediate write mismatch.

    This detects a damaged write; it does not repair old records or certify
    hardware, storage, or later file contents. Strict end-of-run auditing stays.
    """
    def __init__(self, path):
        self.fd = os.open(os.fspath(path), os.O_RDWR | os.O_CREAT | os.O_APPEND, 0o666)
        self.lock = threading.RLock()
        self.closed = False

    def write(self, line):
        data = line.encode('ascii', errors='strict')
        if not data.endswith(b'\n') or data.count(b'\n') != 1:
            raise ValueError('metrics record must be one complete ASCII line')
        with self.lock:
            if self.closed:
                raise ValueError('write to closed metrics log')
            fcntl.flock(self.fd, fcntl.LOCK_EX)
            try:
                start = os.lseek(self.fd, 0, os.SEEK_END)
                remaining = memoryview(data)
                while remaining:
                    written = os.write(self.fd, remaining)
                    if written <= 0:
                        raise OSError('metrics append made no progress')
                    remaining = remaining[written:]
                actual = os.pread(self.fd, len(data), start)
                if actual != data:
                    raise MetricsIntegrityError('metrics append readback mismatch at offset %d' % start)
            finally:
                fcntl.flock(self.fd, fcntl.LOCK_UN)
        return len(line)

    def flush(self):
        with self.lock:
            if not self.closed:
                os.fsync(self.fd)

    def close(self):
        with self.lock:
            if not self.closed:
                os.close(self.fd)
                self.closed = True
