def normalize(s):
    for p in ['?', '.', '.', ',', '!']:
        s = s.replace(p, '')

    return s.lower().strip()
