
from coms import *

import csv
import datetime
import os


if __name__ == "__main__":

    arduino = start_serial(port="COM4")

    data = data_loop(arduino, samples=50)

    headers = ["load", "steps", "displacement"]

    folder_path = "data/loadcell_" + datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = folder_path + "/data.csv"

    if not os.path.exists(folder_path):
        os.makedirs(folder_path)

    with open(csv_path, 'a+', newline='') as f:
        writer = csv.writer(f, delimiter=',')
        writer.writerow(headers)
        [writer.writerow([getattr(s, k) for k in headers]) for s in data]
       
 



