# copernicusnn  

https://console.cloud.google.com/apis/api/language.googleapis.com/overview?project=natlanganalyze&duration=PT1H


https://github.com/ofek/pypinfo/issues/32


https://github.com/etingof/pyasn1-modules/issues/10


https://cloud.google.com/docs/authentication/production#auth-cloud-implicit-python


https://cloud.google.com/docs/authentication/getting-started


https://cloud.google.com/docs/authentication/end-user


https://cloud.google.com/docs/authentication/api-keys


Traceback (most recent call last):  
  File "process.py", line 19, in <module>  
    sentiment, entities = language_analysis(example_text)  
  File "process.py", line 5, in language_analysis  
    client = language.LanguageServiceClient()  
  File "/usr/local/lib/python2.7/dist-packages/google/cloud/language_v1/gapic/language_service_client.py", line 92, in __init__  
    scopes=self._DEFAULT_SCOPES)  
  File "/usr/local/lib/python2.7/dist-packages/google/api_core/grpc_helpers.py", line 132, in create_channel  
    credentials, _ = google.auth.default(scopes=scopes)  
  File "/usr/local/lib/python2.7/dist-packages/google/auth/_default.py", line 283, in default  
    raise exceptions.DefaultCredentialsError(_HELP_MESSAGE)  
google.auth.exceptions.DefaultCredentialsError: Could not automatically determine credentials. Please set GOOGLE_APPLICATION_CREDENTIALS or  
explicitly create credential and re-run the application. For more  
information, please see  
https://developers.google.com/accounts/docs/application-default-credentials.  
