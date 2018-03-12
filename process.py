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
    response = client.analyze_entities(document=document)
    return response.entities


def createEntitiesString(entity):
    entity_type = ('UNKNOWN', 'PERSON', 'LOCATION', 'ORGANIZATION',
                   'EVENT', 'WORK_OF_ART', 'CONSUMER_GOOD', 'OTHER')
    type_s = entity_type[entity.type]
    salience_s = str(entity.salience)
    mentions_s = '\"mentions\" : ['
    for mention in entity.mentions:
    	mentions_s += createMentionsString(mention)
    return_string = '{ \"name\" : ' + entity.name + ', \"entity\" : { \"type\" : ' + type_s + ', \"salience\" : ' + salience_s + ', ' + mentions_s + '  ] } },\n\t\t'
    return return_string


def createMentionsString(mention):
    text_s = '\"text\" : { \"content\" : ' + mention.text.content + ', \"begin_offset\" : ' + str(mention.text.begin_offset) + ' }, '
    type_s = '\"type\" : ' + str(mention.type)		#enum here
    return_string = '{ ' +  text_s + type_s + ' }, '
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
    syntax_tokens = client.analyze_syntax(document=document).tokens
    return syntax_tokens


def createSyntaxString(token):
    text_s = '\"text\" : { \"content\" : ' + token.text.content + ', \"begin_offset\" : ' + str(token.text.begin_offset) + ' }, '
    partOfSpeech_s =  '\"part_of_speech\" : { ' + createPartOfSpeechString(token.part_of_speech) + ' }, '
    dependencyEdge_s = '\"dependency_edge\" : { \"head_token_index\" : ' + str(token.dependency_edge.head_token_index) + ', \"label\" : ' + str(token.dependency_edge.label) + ' }, '			#label is enum
    lemma_s = '\"lemma\" : ' + token.lemma
    return_string = '{ \"syntax\" : {' + text_s + partOfSpeech_s + dependencyEdge_s + ' } }\n'
    return return_string


def createPartOfSpeechString(partOfSpeech):
    return_string = '\"tag\" : ' + str(partOfSpeech.tag) + ', '				# enum
    return_string += '\"aspect\" : ' + str(partOfSpeech.aspect) + ', '			# enum
    return_string += '\"case\" : ' + str(partOfSpeech.case) + ', '			# enum
    return_string += '\"form\" : ' + str(partOfSpeech.form) + ', '			# enum
    return_string += '\"gender\" : ' + str(partOfSpeech.gender) + ', '			# enum
    return_string += '\"mood\" : ' + str(partOfSpeech.mood) + ', '			# enum
    return_string += '\"number\" : ' + str(partOfSpeech.number) + ', '			# enum
    return_string += '\"person\" : ' + str(partOfSpeech.person) + ', '			# enum
    return_string += '\"proper\" : ' + str(partOfSpeech.proper) + ', '			# enum
    return_string += '\"reciprocity\" : ' + str(partOfSpeech.reciprocity) + ', '	# enum
    return_string += '\"tense\" : ' + str(partOfSpeech.tense) + ', '			# enum
    return_string += '\"voice\" : ' + str(partOfSpeech.voice)				# enum
    return return_string


def createJSONLineObject(line):
    sentiment = analyze_sentiment(line)
    entities = analyze_entities(line)
    syntax_tokens = analyze_syntax(line)
    name_s = '{ \"name\" : ' + line 
    sentiment_s = createSentimentString(sentiment)
    entities_s = '{ \"entities\" : {\n\t\t'
    for entity in entities:
        entities_s += createEntitiesString(entity)
    entities_s += '\n\t},\n\t'
    syntax_s = '{ \"syntax\" : {\n\t\t'
    for token in syntax_tokens:
        syntax_s += createSyntaxString(token)
    return name_s + '\n\t' + sentiment_s + entities_s + syntax_s + '},\n'


def writeJsonToFile(jsonDict, fileout, filepath, num):
    path = filepath + fileout + num + '.json'
    file = open(path, "a")
    file.write(jsonDict)
    file.close()


def readFile(filein, fileout, filepath, num):
    filep = filepath + filein + num+ '.txt'
    with open(filep) as fp:  
        line = fp.readline()
        while line:
            jsonDict = createJSONLineObject(line)
            writeJsonToFile(jsonDict, fileout, filepath, num)
            line = fp.readline()


if __name__ == '__main__':
    filein = 'sentences/sentences'
    fileout = 'proc_sent/processed_sentences'
    filepath = '/home/ros/Downloads/copernicusnn-master/'
    readFile(filein, fileout, filepath, str(1))
