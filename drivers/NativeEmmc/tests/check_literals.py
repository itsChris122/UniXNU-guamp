# Fails when a C/C++ string or character literal of the given files contains a raw newline (a "\n" escape that a tool
# turned into a real line break). Comments are skipped; escapes are honoured.
import sys

bad = 0
for path in sys.argv[1:]:
    s = open(path, encoding="utf-8").read()
    i, n, line, strings = 0, len(s), 1, 0
    while i < n:
        c = s[i]
        if s.startswith("//", i):
            j = s.find(chr(10), i)
            i = n if j < 0 else j
            continue
        if s.startswith("/*", i):
            j = s.index("*/", i) + 2
            line += s.count(chr(10), i, j)
            i = j
            continue
        if c in ('"', "'"):
            q = c
            strings += c == '"'
            i += 1
            while i < n and s[i] != q:
                if s[i] == chr(92):
                    i += 1
                elif s[i] == chr(10):
                    print("raw newline in a literal: %s:%d" % (path, line))
                    bad = 1
                    line += 1
                i += 1
            i += 1
            continue
        if c == chr(10):
            line += 1
        i += 1
    print("%s: %d string literals checked" % (path.split("/")[-1], strings))
sys.exit(bad)
