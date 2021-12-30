class MockVar():
    def __init__(self, min, max, delta, nxt_var = None):
        self.min = min
        self.max = max
        self.delta = delta
        self.nxt_var = nxt_var
        self.current = self.min

    def inc_var(self):
        self.current += self.delta
        if self.current > self.max:
            self.current = self.min
            if self.nxt_var != None:
                self.nxt_var.inc_var()

        