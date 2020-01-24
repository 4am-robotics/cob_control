#!/usr/bin/env python
#
# Copyright 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import os
from pylab import figure

EPS = 0.001

DO_REVERSE = True # reverse because stop time is equal for both data - start time could be different

global SAVE_DIR
SAVE_DIR = '/home/fxm-mb/Scripts/Tests/ResultsFmt'

colors = ('b', 'r', 'y', 'g', 'm', 'c', 'k', )

def getMeasData(filename):
    newData = []
    with open(filename) as fp:
        for line in fp.readlines():
            line = line.strip()
            if line.startswith('[') and line.endswith(']'):
                newData.append(eval(line))
    if DO_REVERSE: newData.reverse()
    return newData

def evalData(oldData, newData):
    allErrors = []
    if len(newData) < len(oldData):
        minLen = len(newData)
    else:
        minLen = len(oldData)

    for lv in range(minLen):
        tmpNew = newData[lv]
        tmpOld = oldData[lv]
        jointErrors = []
        for idx, newVal in enumerate(tmpNew):
            errorVal = abs(newVal - tmpOld[idx])
            #if errorVal > EPS:
                # print("Found error greater than eps=%f: In line %d; link %d; error=%f" % (EPS, lv, idx, errorVal))
            jointErrors.append(errorVal)
        allErrors.append(jointErrors)
    if DO_REVERSE: allErrors.reverse()
    return allErrors

def plotErrors(errorData, strTitle):
    errorCnt = len(errorData)
    fig = figure(figsize=(16.0, 10.0))

    positions = []
    if len(errorData[0]) > 7:
        position = 210
    else:
        position = 110
    colorIdx = 0
    for idx in range(len(errorData[0])):
        if 0 == idx % 7:
            position += 1
            positions.append(position)
        colorIdx = idx % 7
        fig.add_subplot(position).stem([elem[idx] for elem in errorData], linefmt=colors[colorIdx] + '-', markerfmt= colors[colorIdx] + 'o', label='joint %d' % idx)

    for position in positions:
        fig.add_subplot(position).set_xlabel('data set number')
        fig.add_subplot(position).set_ylabel('abs. error in rad/s')
        fig.add_subplot(position).set_title(strTitle)
        fig.add_subplot(position).grid(True)
        fig.add_subplot(position).legend()

    fig.subplots_adjust(wspace = 0.2, hspace = 0.5)
    fig.tight_layout()
    if SAVE_DIR is not None:
        fig.savefig(os.path.join(SAVE_DIR, strTitle.replace('.txt', '') + '.eps'))
    # show()

def doEvaluation(filenameNew, filenameOld, titleText):
    newData = getMeasData(filenameNew)
    oldData = getMeasData(filenameOld)
    errorData = evalData(oldData, newData)
    plotErrors(errorData, titleText)

def evaluateFiles(basedir, testdir):
    baseTestDir = os.path.join(baseDir, testDir)
    if not os.path.exists(baseTestDir):
        raise IOError('Could not find directory "%s"' % baseTestDir)

    testResults = []
    for root, dirs, files in os.walk(baseTestDir):
        for file in files:
            if '_new_' in file.lower():
                testResults.append((file, file.replace('_new_', '_old_'), ))

        if len(files) != len(testResults) * 2:
            print('<<<< WARN >>>>: Could not process all files in dir %s. Expected %d files but found %d' % (dirs, len(testResults) * 2, len(files)))

    for testResultNewOld in testResults:
        filepathNew = os.path.join(baseTestDir, testResultNewOld[0])
        filepathOld = os.path.join(baseTestDir, testResultNewOld[1])
        if not os.path.exists(filepathNew):
            print('Could not find path "%s" -> Trying next files for comparison' % filepathNew)
            continue
        if not os.path.exists(filepathOld):
            print('Could not find path "%s" -> Trying next files for comparison' % filepathOld)
            continue

        doEvaluation(filepathNew, filepathOld, testDir + '_' + testResultNewOld[0].replace('_new_', ''))


if __name__ == '__main__':
    baseDir = "/home/fxm-mb/Scripts/Tests/"
    for root, dirs, files in os.walk(baseDir):
        for testDir in dirs:
            if True: # 'manip_none' in testDir:
                print('Evaluating files in dir %s' % testDir)
                evaluateFiles(baseDir, testDir)
