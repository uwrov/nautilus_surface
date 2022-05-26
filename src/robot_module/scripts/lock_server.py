#!/usr/bin/env python3

from locks.lock_service_server import LockServer

if __name__ == '__main__':
    ls = LockServer()
    ls.init_server()