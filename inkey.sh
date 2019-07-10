sleep 2
input keyevent 26
sleep 2
input keyevent 26
sleep 1
sqlite3 /data/data/com.android.providers.settings/databases/settings.db "INSERT INTO system VALUES(null,'screen_off_timeout','2147483647');"
sqlite3 /data/system/locksettings.db "update locksettings set value=1 where name='lockscreen.disabled'"
