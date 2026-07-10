#!/usr/bin/env python3
"""Точка входа отладочного GUI для FW_SonarMotorDriver.

Запуск:  python run.py
"""
import sys

from sonar_gui.app import main

if __name__ == "__main__":
    sys.exit(main())
