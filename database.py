


import sqlite3
conn = sqlite3.connect('hit_record.db')
c = conn.cursor()
c.execute('''CREATE TABLE  hit_record (head_way_noise,hit_num,cur_time,avg_speed,s0,T)''')
conn.commit()
conn.close()

# import sqlite3
# conn = sqlite3.connect('hit_record.db')
# c = conn.cursor()
# c.execute('''INSERT INTO  hit_record (head_way_noise,hit_num,cur_time,route_len,car_num)
#             values(1,1,1)''')
# conn.commit()
# conn.close()


# import sqlite3
# conn = sqlite3.connect('hit_record.db')
# c = conn.cursor()
# c.execute('''select * from hit_record''')
# print(c.fetchall())
# conn.close()
