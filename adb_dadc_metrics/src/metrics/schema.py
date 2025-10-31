#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from dataclasses import dataclass
from typing import Dict, Optional

@dataclass
class ColumnMap:
    mapping: Dict[str, str]
    def get(self, key: str) -> Optional[str]:
        return self.mapping.get(key)
