from Database import database

data = database('tuananhvam','vam123')

data.connect()

new_data = {
    'name': 'John Doe',
    'age': 25,
    'email': 'johndoe@example.com'
}

data_pub = data.pub_data(new_data, 'your_database','your_collection')

print(data_pub)

get_data = data.get_data('your_database','your_collection')

for document in get_data:
    print(document)
    a = document
print(document)
print(get_data)