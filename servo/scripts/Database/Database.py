# <your-connection-string>: mongodb+srv://<username>:<password>@vam.gqgjopp.mongodb.net/test
# <username>: tuananhvam
# <password>: vam123

from pymongo import MongoClient

class database:
    def __init__(self, User, Password):
        self.Parameter = {'username' : User,
                          'password' : Password,
                          }

    def get_data(self,name_db,name_cl):
        if self.check_connect == 1:
            try:
                self.db = self.client[name_db]
                self.collection = self.db[name_cl]
                self.data = self.collection.find()
                print("Get data from database complete")
            except:
                print("ErrCode: 002. No get data from database")
                self.data = []
        else:
            print("ErrCode: 001. Miss connect database. Need run connect() befor run get_data().")
            self.data = []

        return self.data


    def pub_data(self,data,name_db,name_cl):
        if self.check_connect == 1:
            try:
                self.db = self.client[name_db]
                self.collection = self.db[name_cl]
                self.result = self.collection.insert_one(data)
                print("Publish data to database complete.")
            except:
                print("ErrCode: 002. No publish data to database")
                return
        else:
            print("ErrCode: 001. Miss connect database. Need run connect() befor run get_data().")
            return
        # Return the inserted document's ID
        return self.result.inserted_id


    def connect(self):
        try:
            self.client = MongoClient('mongodb+srv://' + self.Parameter['username'] + ':' + self.Parameter['password'] + '@cluster0.7zqsaa7.mongodb.net/DT')
            self.check_connect = 1
            print("Connect Database complete")
        except:
            self.check_connect = 0
            print("ErrCode: 000. No connect with Database. Please try again")