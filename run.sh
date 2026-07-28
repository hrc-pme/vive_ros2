#!/bin/bash
# HRCLAB Vive Tracker Docker 一鍵啟動腳本


cd "$(dirname "$0")"


docker compose run --rm vive_tracker bash