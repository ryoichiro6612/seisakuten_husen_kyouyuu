import sqlite3
conn = sqlite3.connect('husen_kansou.db')

c = conn.cursor()

c.execute('''CREATE TABLE kansou
             (husen_id integer,sakuhin_id integer, time_stamp datetime)''')

# Insert a row of data
c.execute("INSERT INTO kansou VALUES (4, 1,'2006-01-05 12:11:11')")

# Save (commit) the changes
conn.commit()

# We can also close the connection if we are done with it.
# Just be sure any changes have been committed or they will be lost.
conn.close()
