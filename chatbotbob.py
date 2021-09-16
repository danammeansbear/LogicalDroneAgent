# Importing chatterbot
from chatterbot import ChatBot

# Create object of ChatBot class
bot = ChatBot('Buddy')

# Create object of ChatBot class with Storage Adapter
#bot = ChatBot(
#    'Buddy',
#    storage_adapter='chatterbot.storage.SQLStorageAdapter',
#    database_uri='sqlite:///database.sqlite3'
#)

# Create object of ChatBot class with Logic Adapter
bot = ChatBot(
    'Buddy',  
    logic_adapters=[
        'chatterbot.logic.BestMatch',
        'chatterbot.logic.TimeLogicAdapter'],
)