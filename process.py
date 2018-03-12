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
    mention_type = ('TYPE_UNKNOWN', 'PROPER', 'COMMON')
    text_s = '\"text\" : { \"content\" : ' + mention.text.content + ', \"begin_offset\" : ' + str(mention.text.begin_offset) + ' }, '
    type_s = '\"type\" : ' + mention_type[mention.type]		#enum here
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
    return_string = '\n\t{ \"sentiment\" : { \"score\" : ' + str(score) + ', "magnitude\" : ' + str(magnitude) + ' } },' 
    return return_string


def analyze_syntax(text):
    client = language.LanguageServiceClient()
    document = types.Document(content=text, type=enums.Document.Type.PLAIN_TEXT)
    syntax_tokens = client.analyze_syntax(document=document).tokens
    return syntax_tokens


def createSyntaxString(token):
    text_s = '\n\t\t\t{\n\t\t\t\t\"text\" : { \"content\" : ' + token.text.content + ', \"begin_offset\" : ' + str(token.text.begin_offset) + ' }, '
    partOfSpeech_s =  '\n\t\t\t\t\"part_of_speech\" : { ' + createPartOfSpeechString(token.part_of_speech) + ' }, '
    de_label = ('UNKNOWN', 'ABBREV', 'ACOMP', 'ADVCL', 'ADVMOD', 'AMOD', 'APPOS', 'ATTR', 'AUX', 'AUXPASS', 'CC', 'CCOMP', 'CONJ', 'CSUBJ', 'CSUBJPASS', 'DEP', 'DET', 'DISCOURSE', 'DOBJ', 'EXPL', 'GOESWITH', 'IOBJ', 'MARK', 'MWE', 'MWV', 'NEG', 'NN', 'NPADVMOD', 'NSUBJ', 'NSUBJPASS', 'NUM', 'NUMBER', 'P', 'PARATAXIS', 'PARTMOD', 'PCOMP', 'POBJ', 'POSS', 'POSTNEG', 'PRECOMP', 'PRECONJ', 'PREDET', 'PREF', 'PREP', 'PRONL', 'PRT', 'PS', 'QUANTMOD', 'RCMOD', 'RCMODREL', 'RDROP', 'REF', 'REMNANT', 'REPARANDUM', 'ROOT', 'SNUM', 'SUFF', 'TMOD', 'TOPIC', 'VMOD', 'VOCATIVE', 'XCOMP', 'SUFFIX', 'TITLE', 'ADVPHMOD', 'AUXCAUS', 'AUXVV', 'DTMOD', 'FOREIGN', 'KW', 'LIST', 'NOMC', 'NOMCSUBJ', 'NOMCSUBJPASS', 'NUMC', 'COP', 'DISLOCATED', 'ASP', 'GMOD', 'GOBJ', 'INFMOD', 'MES', 'NCOMP')
    dependencyEdge_s = '\n\t\t\t\t\"dependency_edge\" : { \"head_token_index\" : ' + str(token.dependency_edge.head_token_index) + ', \"label\" : ' + de_label[token.dependency_edge.label] + ' },'			#label is enum
    lemma_s = '\n\t\t\t\t\"lemma\" : ' + token.lemma
    return_string = '\n\t\t{ \"token\" : ' + text_s + partOfSpeech_s + dependencyEdge_s + lemma_s + ' },\n'
    return return_string


def createPartOfSpeechString(partOfSpeech):
    # part-of-speech tags from enums.PartOfSpeech.Tag
    pos_tag = ('UNKNOWN', 'ADJ', 'ADP', 'ADV', 'CONJ', 'DET', 'NOUN', 'NUM',
               'PRON', 'PRT', 'PUNCT', 'VERB', 'X', 'AFFIX')
    return_string = '\"tag\" : ' + pos_tag[partOfSpeech.tag] + ', '				# enum
    pos_aspect = ('ASPECT_UNKNOWN', 'PERFECTIVE', 'IMPERFECTIVE', 'PROGRESSIVE')
    return_string += '\"aspect\" : ' + pos_aspect[partOfSpeech.aspect] + ', '			# enum
    pos_case = ('CASE_UNKNOWN', 'ACCUSATIVE', 'ADVERBAL', 'COMPLEMENTIVE', 'DATIVE', 'GENITIVE', 'INSTRUMENTAL', 'LOCATIVE', 'NOMINATIVE', 'OBLIQUE', 'PARTITIVE', 'PREPOSTIONAL', 'REFLEXIVE_CASE', 'RELATIVE_CASE', 'VOCATIVE')
    return_string += '\"case\" : ' + pos_case[partOfSpeech.case] + ', '			# enum
    pos_form = ('FORM_UNKNOWN', 'ADNOMIAL', 'AUXILIARY', 'COMPLEMENTIZER', 'FINAL_ENDING', 'GERUND', 'REALIS', 'IRREALIS', 'SHORT', 'LONG', 'ORDER', 'SPECIFIC')
    return_string += '\"form\" : ' + pos_form[partOfSpeech.form] + ', '			# enum
    pos_gender = ('GENDER_UNKNOWN', 'FEMININE', 'MASCULINE', 'NEUTER')
    return_string += '\"gender\" : ' + pos_gender[partOfSpeech.gender] + ', '			# enum
    pos_mood = ('MOOD_UNKNOWN', 'CONDITIONAL_MOOD', 'IMPERATIVE', 'INDICATIVE', 'INTERROGATIVE', 'JUSSIVE', 'SUBJUNCTIVE')
    return_string += '\"mood\" : ' + pos_mood[partOfSpeech.mood] + ', '			# enum
    pos_number = ('NUMBER_UNKNOWN', 'SINGULAR', 'PLURAL', 'DUAL')
    return_string += '\"number\" : ' + pos_number[partOfSpeech.number] + ', '			# enum
    pos_person = ('PERSON_UNKNOWN', 'FIRST', 'SECOND', 'THIRD', 'REFLEXIVE_PERSON')
    return_string += '\"person\" : ' + pos_person[partOfSpeech.person] + ', '			# enum
    pos_proper = ('PROPER_UNKNOWN', 'PROPER', 'NOT_PROPER')
    return_string += '\"proper\" : ' + pos_proper[partOfSpeech.proper] + ', '			# enum
    pos_reciprocity = ('RECIPROCITY_UNKNOWN', 'RECIPROCAL', 'NON_RECIPROCAL')
    return_string += '\"reciprocity\" : ' + pos_reciprocity[partOfSpeech.reciprocity] + ', '	# enum
    pos_tense = ('TENSE_UNKNOWN', 'CONDITIONAL_TENSE', 'FUTURE', 'PAST', 'PRESENT', 'IMPERFECT', 'PLUPERFECT')
    return_string += '\"tense\" : ' + pos_tense[partOfSpeech.tense] + ', '			# enum
    pos_voice = ('VOICE_UNKNOWN', 'ACTIVE', 'CAUSATIVE', 'PASSIVE')
    return_string += '\"voice\" : ' + pos_voice[partOfSpeech.voice]				# enum
    return return_string


def createJSONLineObject(line):
    sentiment = analyze_sentiment(line)
    entities = analyze_entities(line)
    syntax_tokens = analyze_syntax(line)
    name_s = '{ \"name\" : ' + line 
    sentiment_s = createSentimentString(sentiment)
    entities_s = '\n\t{ \"entities\" : {\n\t\t'
    for entity in entities:
        entities_s += createEntitiesString(entity)
    entities_s += '\n\t},'
    syntax_s = '\n\t{ \"syntax\" : ['
    for token in syntax_tokens:
        syntax_s += createSyntaxString(token)
    return_string = name_s + '\n\t' + sentiment_s + entities_s + syntax_s + '\n\t]}\n},\n'
    return return_string.encode('UTF-8')


def writeJsonToFile(jsonDict, fileout, filepath, num):
    path = filepath + fileout + num + '.json'
    file = open(path, "a")
    file.write(jsonDict)
    file.close()


def readFile(filein, fileout, filepath, num):
    count = 1
    filep = filepath + filein + num+ '.txt'
    with open(filep, encoding="utf8") as fp:  
        line = fp.readline()
        while line:
            print(count)
            jsonDict = createJSONLineObject(line)
            writeJsonToFile(jsonDict, fileout, filepath, num)
            count += 1
            line = fp.readline()


if __name__ == '__main__':
    filein = 'sentences/sentences'
    fileout = 'proc_sent/processed_sentences'
    filepath = '/home/ros/Downloads/copernicusnn-master/'
    readFile(filein, fileout, filepath, str(1))
