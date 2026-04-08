from __future__ import annotations

import socket
from abc import ABC, abstractmethod

from .config import ConnectionConfig, TransportKind


class BaseTransport(ABC):
    def __init__(self, config: ConnectionConfig) -> None:
        self.config = config

    @abstractmethod
    def connect(self) -> None:
        raise NotImplementedError

    @abstractmethod
    def close(self) -> None:
        raise NotImplementedError

    @abstractmethod
    def send(self, data: bytes) -> None:
        raise NotImplementedError

    @abstractmethod
    def recv(self, size: int = 4096) -> bytes:
        raise NotImplementedError


class TcpTransport(BaseTransport):
    def __init__(self, config: ConnectionConfig) -> None:
        super().__init__(config)
        self._sock: socket.socket | None = None

    def connect(self) -> None:
        self._sock = socket.create_connection(
            (self.config.host, self.config.tcp_port),
            timeout=self.config.connect_timeout,
        )
        self._sock.settimeout(self.config.recv_timeout)

    def close(self) -> None:
        if self._sock is not None:
            try:
                self._sock.close()
            finally:
                self._sock = None

    def send(self, data: bytes) -> None:
        if self._sock is None:
            raise RuntimeError("transport is not connected")
        self._sock.sendall(data)

    def recv(self, size: int = 4096) -> bytes:
        if self._sock is None:
            raise RuntimeError("transport is not connected")
        return self._sock.recv(size)


class UdpTransport(BaseTransport):
    def __init__(self, config: ConnectionConfig) -> None:
        super().__init__(config)
        self._sock: socket.socket | None = None

    def connect(self) -> None:
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind((self.config.local_host, self.config.local_port))
        self._sock.connect((self.config.host, self.config.udp_port))
        self._sock.settimeout(self.config.recv_timeout)

    def close(self) -> None:
        if self._sock is not None:
            try:
                self._sock.close()
            finally:
                self._sock = None

    def send(self, data: bytes) -> None:
        if self._sock is None:
            raise RuntimeError("transport is not connected")
        self._sock.send(data)

    def recv(self, size: int = 4096) -> bytes:
        if self._sock is None:
            raise RuntimeError("transport is not connected")
        return self._sock.recv(size)


def build_transport(config: ConnectionConfig) -> BaseTransport:
    if config.transport == TransportKind.TCP:
        return TcpTransport(config)
    return UdpTransport(config)
