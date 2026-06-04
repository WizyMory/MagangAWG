import csv, os
from datetime import datetime

class RelayCSVLogger:
    def __init__(self, log_dir):
        os.makedirs(log_dir, exist_ok=True)
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.file = open(
            os.path.join(log_dir, f'relay_{ts}.csv'), 'w', newline=''
        )
        self.writer = csv.writer(self.file)
        self.writer.writerow(['time', 'channel', 'state', 'result'])

    def log(self, ch, state, result):
        self.writer.writerow([
            datetime.now().isoformat(),
            ch,
            'ON' if state else 'OFF',
            result
        ])
        self.file.flush()

    def close(self):
        self.file.close()
