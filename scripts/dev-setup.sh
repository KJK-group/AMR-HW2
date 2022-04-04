#!/usr/bin/env bash
# -*- coding: utf-8 -*-

set -eu -o pipefail

if ! hash pre-commit; then
	echo "installing pre-commit"
	pip install -u pre-commit
	echo "installing pre-commit done"
fi

pre-commit install
