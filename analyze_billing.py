#!/usr/bin/python

import argparse

_cmdline_desc = """\
Analyze output of rountrip tests.

First run: roundtriptest/run.sh &> output
Then pass output as an argument to this script.
"""

_cmdline_parser = argparse.ArgumentParser(
    formatter_class=argparse.RawDescriptionHelpFormatter,
    description='Analyze output of h264dec')
_cmdline_parser.add_argument(
    '-v',
    '--verbose',
    action='store_true',
    help='Print debugging statements.',
)
_cmdline_parser.add_argument(
    'output_path',
    type=str,
    help='Path to stdout file capture of h264dec.',
)

from collections import OrderedDict
import re
feature_bill_re = re.compile(r'^(?P<index>\d*)\s*::\s*(?P<bytes>\d+(.\d+)?)\s*\[(?P<label>.*)\]')

HEADER = ('Feature', 'Bench', 'Ours', 'O-B', 'O/B')

header_row_format = '{:>22} {:>10} {:>10} {:>10} {:>10}%'
feature_row_format = '{:>22} {:>10,} {:>10,} {:>10,} {:>10}%'

class VideoCompressionResult(object):
    def __init__(self, name):
        self.name = name
        self.ours = OrderedDict()
        self.benchmark = OrderedDict()
    def __repr__(self):
        return 'VideoCompressionResult(%r)' % self.name

def parse_outputs(output_path):
    video_files = []
    video_result = VideoCompressionResult('video file')
    video_results = [video_result]
    with open(output_path, 'r') as output_file:
        for line in output_file:
            if line.startswith('==='):
                video_name = line.strip(' =\n')
                video_files.append(video_name)
                video_result = VideoCompressionResult(video_name)
                video_results.append(video_result)
                continue
            m = feature_bill_re.match(line.strip())
            if m:
                label = m.groupdict()['label']
                if '.' in m.groupdict()['bytes']:
                    video_result.benchmark[label] = int(float(m.groupdict()['bytes']))
                else:
                    video_result.ours[label] = int(m.groupdict()['bytes'])
                continue
    return [video_result for video_result in video_results
            if video_result.ours or video_result.benchmark]

def perc(num, denom):
    if denom == 0:
        return float('inf')
    else:
        return int(round(100 * float(num) / denom))

def analyze(output_path):
    video_results = parse_outputs(output_path)
    for video_result in video_results:
        print 'Analysis of %s' % video_result.name
        print '\t' + header_row_format.format(*HEADER)

        # Combine the label lists preserving order.
        ours_labels = {k:i for i, k in enumerate(video_result.ours.keys())}
        benchmark_labels = {k:i for i, k in enumerate(video_result.benchmark.keys())}
        def order(a, b):
            if a in ours_labels and b in ours_labels:
                return cmp(ours_labels[a], ours_labels[b])
            if a in benchmark_labels and b in benchmark_labels:
                return cmp(benchmark_labels[a], benchmark_labels[b])
            return 0
        labels = sorted(set(ours_labels) | set(benchmark_labels), cmp=order)

        for label in labels:
            bench = video_result.benchmark.get(label, 0)
            us = video_result.ours.get(label, 0)
            print '\t' + feature_row_format.format(label, bench, us, us - bench, perc(us, bench))

        for prefix in 'luma ', 'chroma ', 'pred ', '':
            def total(table):
                return sum(v for k, v in table.items() if k.startswith(prefix))
            bench = total(video_result.benchmark)
            us = total(video_result.ours)
            print '\t' + feature_row_format.format(
                    '{}total:'.format(prefix), bench, us, us - bench, perc(us, bench))

        print ''
    return video_results

def main():
    parsed_args = _cmdline_parser.parse_args()
    return analyze(parsed_args.output_path)

if __name__ == '__main__':
    video_results = main()

