# Run h264dec on a single file compiled with PRIOR_STATS and then run this script
# Outputs timeseries plot at /tmp/misses.pdf
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
import os

def temporal_misses(key):
    values = data[key]
    numbins = 100
    binsize = len(values) // numbins
    bins = [[]]
    for v in values:
        if len(bins[-1]) >= binsize:
            bins.append([])
        bins[-1].append(v)

    x = range(len(bins))
    total_misses = float(sum(values))
    y = [100 * float(sum(b)) / total_misses for b in bins]
    return plt.plot(x, y, label=key)[0]

paths = filter(lambda s: 'misses.log' in s, os.listdir('/tmp/'))
data = {p.split('_misses.')[0]: map(lambda c: c == '0', open('/tmp/' + p).read()) for p in paths}
handles = []
plt.figure(figsize=(20,10))
keys = data.keys()
for k in keys:
    handles.append(temporal_misses(k))
plt.axis((0, 100, 0, 2))
plt.xlabel('temporal %')
plt.ylabel('% total misses')
plt.legend(handles, keys, bbox_to_anchor=(1, 1), bbox_transform=plt.gcf().transFigure)

out = PdfPages('/tmp/misses.pdf')
out.savefig()
out.close()
