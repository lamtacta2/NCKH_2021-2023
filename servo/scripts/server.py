import pymongo
from urllib.parse import quote_plus
username = quote_plus('tta')
password = quote_plus('231000')
cluster = 'Cluster0'
authSource = '<authSource>'
authMechanism = '<authMechanism>'
uri = 'mongodb+srv://' + username + ':' + password + '@' + cluster + '/?authSource=' + authSource + '&authMechanism=' + authMechanism
a = 'mongodb+srv://tta:<password>@cluster0.7zqsaa7.mongodb.net/'
client = pymongo.MongoClient(a)
result = client["<dbName"]["<collName>"].find()
# print results
for i in result:
    print(i)