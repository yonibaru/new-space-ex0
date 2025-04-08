class Moon:
    # Moon characteristics (from Wikipedia)
    RADIUS = 3475 * 1000  # meters
    ACC = 1.622         # m/s², gravitational acceleration on the Moon
    EQ_SPEED = 1700     # m/s, some reference speed

    @staticmethod
    def get_acc(speed):
        """Compute the effective lunar gravitational acceleration based on horizontal speed."""
        n = abs(speed) / Moon.EQ_SPEED
        ans = (1 - n) * Moon.ACC
        return ans