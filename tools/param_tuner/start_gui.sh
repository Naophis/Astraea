#!/bin/bash

cd "$(dirname "$0")"

echo "ESP32-S3 Parameter Tuner GUI を起動します..."

# Node modulesがインストールされているか確認
if [ ! -d "node_modules" ]; then
    echo "依存関係をインストールしています..."
    npm install
fi

# Electronを起動
npm start
