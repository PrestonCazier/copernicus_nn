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
	

def analyze_entities(text):
    client = language.LanguageServiceClient()
    document = types.Document(content=text, type=enums.Document.Type.PLAIN_TEXT)
    entities = client.analyze_entities(document=document)
    return entities


def createEntitiesString(entities):
    type_s = entities.type
    salience_s = entities.salience
    mentions_s = createMentionsString(entities.mentions)
    return_string = '{ \"sentiment\" : { \"type\" : ' + type_s + ', \"salience\" : ' + salience_s + ', \"mentions\" : [ { ' + mentions_s + ' } ] } },\n\t'


def createMentionsString(mentions):
    text_s = '\"text\" : { \"content\" : ' + mentions.text.content + ', \"beginOffset\" : ' + mentions.text.beginOffset + ' }, '
    type_s = '\"type\" : ' + mentions.type
    return_string = text_s + type_s
    return return_string


def analyze_sentiment(text):
    client = language.LanguageServiceClient()
    document = types.Document(content=text, type=enums.Document.Type.PLAIN_TEXT)
    response = client.analyze_sentiment(document=document)
    sentiment = response.document_sentiment
    return sentiment


def createSentimentString(sentiment):
    score = sentiment.score
    magnitude = sentiment.magnitude
    return_string = '{ \"sentiment\" : { \"score\" : ' + str(score) + ', "magnitude\" : ' + str(magnitude) + ' } },\n\t' 
    return return_string


def analyze_syntax(text):
    client = language.LanguageServiceClient()
    document = types.Document(content=text, type=enums.Document.Type.PLAIN_TEXT)
    syntax = client.analyze_syntax(document=document)
    return syntax


def createSyntaxString(syntax):
    text_s = '\"text\" : { \"content\" : ' + syntax.text.content + ', \"beginOffset\" : ' + syntax.text.beginOffset + ' }, '
    partOfSpeech_s =  '\"partOfSpeech\" : { ' + createPartOfSpeechString(syntax.partOfSpeech) + ' }, '
    dependencyEdge_s = '\"dependencyEdge\" : { \"headTokenIndex\" : ' + syntax.dependencyEdge.headTokenIndex + ', \"label\" : ' + syntax.dependencyEdge.label + ' }, '
    lemma_s = '\"lemma\" : ' + syntax.lemma
    return_string = '{ \"syntax\" : {' + text_s + partOfSpeech_s + dependencyEdge_s + ' } }\n'
    return return_string


def createPartOfSpeechString(partOfSpeech):
    return_string = '\"tag\" : ' + partOfSpeech.Tag + ', '
    return_string += '\"aspect\" : ' + partOfSpeech.Aspect + ', '
    return_string += '\"case\" : ' + partOfSpeech.Case + ', '
    return_string += '\"form\" : ' + partOfSpeech.Form + ', '
    return_string += '\"gender\" : ' + partOfSpeech.Gender + ', '
    return_string += '\"mood\" : ' + partOfSpeech.Mood + ', '
    return_string += '\"number\" : ' + partOfSpeech.Number + ', '
    return_string += '\"person\" : ' + partOfSpeech.Person + ', '
    return_string += '\"proper\" : ' + partOfSpeech.Proper + ', '
    return_string += '\"reciprocity\" : ' + partOfSpeech.Reciprocity + ', '
    return_string += '\"tense\" : ' + partOfSpeech.Tense + ', '
    return_string += '\"voice\" : ' + partOfSpeech.Voice
    return return_string


def createJSONLineObject(line):
    sentiment = analyze_sentiment(line)
    entities = analyze_entities(line)
    syntax = analyze_syntax(line)
    name_s = '{ \"name\" : ' + line 
    sentiment_s = createSentimentString(sentiment)
    entities_s = createEntitiesString(entities)
    syntax_s = createSyntaxString(syntax)
    return name_s + '\n\t' + sentiment_s + entities_s + syntax_s + '},\n'


def writeJsonToFile(jsonDict, fileout, filepath, num):
    path = filepath + fileout + num + '.json'
    file = open(path, "a")
    file.write(jsonDict)
    file.close()


def readFile(filein, fileout, filepath, num):
    filepath = filepath + filein + num+ '.txt'
    with open(filepath) as fp:  
        line = fp.readline()
        while line:
            jsonDict = createJSONLineObject(line)
            writeJsonToFile(jsonDict, fileout, num)
            line = fp.readline()


if __name__ == '__main__':
    filein = 'sentences/sentences'
    fileout = 'proc_sent/processed_sentences'
    filepath = '/home/ros/Downloads/copernicusnn-master/'
    readFile(filein, fileout, filepath, str(1))
