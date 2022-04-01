#!/usr/bin/env python
# -*- coding: utf-8 -*-

import subprocess
import os
import argparse
from google.cloud import language_v1 as language
# https://cloud.google.com/natural-language

def main(credentials_path, voice_text):
    if credentials_path:
        if not os.path.exists(credentials_path):
            raise Exception("file {} is not found, please specify existing google cloud credentials json file path".format(credentials_path))
        os.environ['GOOGLE_APPLICATION_CREDENTIALS'] = credentials_path

        client = language.LanguageServiceClient()
        try:
            document = language.types.Document (
                content = voice_text,
                type = 'PLAIN_TEXT'
            )
            response = client.analyze_entities (
                document = document,
                encoding_type = 'UTF32'
            )
            # Detects the entities and sentiment of the text
            for entity in response.entities:
                print("name:      ", entity.name.encode('utf-8'))
                print("type:      ", entity.type)
                print("metadata:  ", entity.metadata.items())
                print("salience:  ", entity.salience)
            # Detects the sentiment of the text
            sentiment = client.analyze_sentiment (
                document = document,
                encoding_type = 'UTF32'
            ).document_sentiment
            print(" ---------- ")
            print("score:     ", sentiment.score)
            print("magnitude: ", sentiment.magnitude)

            subprocess.call(["curl", "-X", "POST", "--data-urlencode", "degree=" + str(sentiment.score * 20), "http://localhost:3000/api/info"])
        
        except Exception as e:
            print("Fail to analyze syntax ... {}".format(str(e)))

parser = argparse.ArgumentParser()
parser.add_argument("--credentials-path")
parser.add_argument("--voice-text")
args = parser.parse_args()
main(args.credentials_path, args.voice_text)