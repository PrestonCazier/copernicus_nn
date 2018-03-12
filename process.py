#!/usr/bin/env python

# Copyright 2016 Google, Inc
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Analyzes text using the Google Cloud Natural Language API."""

import argparse
import json
import sys

#import googleapiclient.discovery
from google.cloud import language
from google.cloud.language import enums
from google.cloud.language import types


def get_native_encoding_type():
	"""Returns the encoding type that matches Python's native strings."""
	if sys.maxunicode == 65535:
		return 'UTF16'
	else:
		return 'UTF32'
	

def analyze_entities_single_sentence(text):
    client = language.LanguageServiceClient()
    document = types.Document(content=text, type=enums.Document.Type.PLAIN_TEXT)
    entities = client.analyze_entities(document=document)
    return entities


def analyze_sentiment(text):
    client = language.LanguageServiceClient()
    document = types.Document(content=text, type=enums.Document.Type.PLAIN_TEXT)
    sentiment = client.analyze_sentiment(document=document)
    return sentiment


def analyze_syntax(text):
    client = language.LanguageServiceClient()
    document = types.Document(content=text, type=enums.Document.Type.PLAIN_TEXT)
    syntax = client.analyze_syntax(document=document)
    return syntax


def createJSONLineObject(line):
    sentiment = analyze_sentiment_single_sentence(line)
    entities = analyze_entities_single_sentence(line)
    syntax = analyze_syntax_single_sentence(line)
    #print(sentiment)
    #print(entities)
    #print(syntax)
    #print("Line {}: {}".format(cnt, line.strip()))
    pythonDictionary = {'sentiment' : sentiment, 'entities' : entities, 'syntax' : syntax }
    return pythonDictionary


def writeJsonToFile(jsonDict, fileout, filepath, num):
    path = filepath + fileout + num + '.json'
    file = open(path, "a")
    file.write(jsonDict)
    file.close()


def readFile(filein, fileout, filepath):
    #for x in range(1, 100)
    filepath = filepath + filein + '1.txt'
    with open(filepath) as fp:  
        line = fp.readline()
        while line:
            dictionaryToJson = json.dumps(createJSONLineObject(line))
            writeJsonToFile(dictionaryToJson, fileout)
            line = fp.readline()
    return


if __name__ == '__main__':
    filein = 'sentences'
    fileout = 'processed_sentences'
    filepath = '/home/ros/Downloads/copernicusnn-master/'
    readFile(filein, fileout, filepath)
