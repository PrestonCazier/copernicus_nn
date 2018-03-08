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

#def language_analysis(text):
#	client = language.LanguageServiceClient()
#	document = client.document_from_text(text)
#	sent_analysis = document.analyze_sentiment()
#	dir(sent_analysis)
#	sentiment = sent_analysis.sentiment
#
#	ent_analysis = document.analyze_entities()
#	dir(ent_analysis)
#	entities = ent_analysis.entities
#
#	return sentiment, entities
#
#
#example_text = 'Python is such a great programming language'
#sentiment, entities = language_analysis(example_text)
#print(sentiment.score, sentiment.magnitude)
#for e in entities:
#	print(e.name, e.entity_type, e.metadata, e.salience)

	
def get_native_encoding_type():
	"""Returns the encoding type that matches Python's native strings."""
	if sys.maxunicode == 65535:
		return 'UTF16'
	else:
		return 'UTF32'
	
#def analyze_entities(text, encoding='UTF32'):
#    body = {
#        'document': {
#            'type': 'PLAIN_TEXT',
#            'content': text,
#        },
#        'encoding_type': encoding,
#    }
#
#    service = googleapiclient.discovery.build('language', 'v1')
#
#    request = service.documents().analyzeEntities(body=body)
#    response = request.execute()
#
#    return response
def analyze_entities_single_sentence(text):
	client = language.LanguageServiceClient()

    document = types.Document(
        content=text,
        type=enums.Document.Type.PLAIN_TEXT)

    entities = client.analyze_entities(document=document)

    return entities


def analyze_entities_paragraph(text):
	client = language.LanguageServiceClient()

    document = types.Document(
        content=text,
        type=enums.Document.Type.PLAIN_TEXT)

    entities = client.analyze_entities(document=document)

    return entities


#def analyze_sentiment(text, encoding='UTF32'):
#    body = {
#        'document': {
#            'type': 'PLAIN_TEXT',
#            'content': text,
#        },
#        'encoding_type': encoding
#    }
#
#    service = googleapiclient.discovery.build('language', 'v1')
#
#    request = service.documents().analyzeSentiment(body=body)
#    response = request.execute()
#
#    return response
def analyze_sentiment_single_sentence(text):
    client = language.LanguageServiceClient()

    document = types.Document(
        content=text,
        type=enums.Document.Type.PLAIN_TEXT)

    sentiment = client.analyze_sentiment(document=document)

    return sentiment


def analyze_sentiment_paragraph(text):
    client = language.LanguageServiceClient()

    document = types.Document(
        content=text,
        type=enums.Document.Type.PLAIN_TEXT)

    sentiment = client.analyze_sentiment(document=document)

    return sentiment


#def analyze_syntax(text, encoding='UTF32'):
#    body = {
#        'document': {
#            'type': 'PLAIN_TEXT',
#            'content': text,
#        },
#        'encoding_type': encoding
#    }
#
#    service = googleapiclient.discovery.build('language', 'v1')
#
#    request = service.documents().analyzeSyntax(body=body)
#    response = request.execute()
#
#    return response
def analyze_syntax_single_sentence(text):
	client = language.LanguageServiceClient()

    document = types.Document(
        content=text,
        type=enums.Document.Type.PLAIN_TEXT)

    syntax = client.analyze_syntax(document=document)

    return syntax


def analyze_syntax_paragraph(text):
	client = language.LanguageServiceClient()

    document = types.Document(
        content=text,
        type=enums.Document.Type.PLAIN_TEXT)

    syntax = client.analyze_syntax(document=document)

    return syntax


def looper()
    sample_text = '''Here is some sample text.  Aaron shouldn't sleep in class.  Boosting is a principle used in many applications for aid in in image processing.'''
    encoding = get_native_encoding_type()
    sentiment = analyze_sentiment_paragraph(sample_text, encoding)
    entities = analyze_entities_paragraph(sample_text, encoding)
    syntax = analyze_syntax_paragraph(sample_text, encoding)
    print(sentiment)
    print(entities)
    print(syntax)


if __name__ == '__main__':
    looper()
