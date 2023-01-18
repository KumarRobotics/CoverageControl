import os.path
import ujson

from OSMPythonTools.cachingStrategy.base import CachingStrategyBase

class CachingNone(CachingStrategyBase):
    def __init__(self, cacheDir='cache'):
        self._cacheDir = cacheDir

    def _filename(self, key):
        return os.path.join(self._cacheDir, key)

    def get(self, key):
        filename = self._filename(key)
        data = None
        return data

    def set(self, key, value):
        return
