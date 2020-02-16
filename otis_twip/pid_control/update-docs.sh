#!/bin/sh
set -e

rm -rf target
cargo doc -p pid_control || cargo doc -p pid_control
gittar -b gh-pages file:target/doc/*
git push origin gh-pages
