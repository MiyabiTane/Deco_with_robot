from selenium import webdriver
from webdriver_manager.chrome import ChromeDriverManager

import time

def get_driver(url):
    chrome_options = webdriver.ChromeOptions()
    chrome_options.add_argument('--incognito')
    chrome_options.add_experimental_option("excludeSwitches", ['enable-automation'])
 
    driver = webdriver.Chrome(ChromeDriverManager().install(), options=chrome_options)
    driver.set_page_load_timeout(600)
    driver.implicitly_wait(2)
    driver.set_script_timeout(600)

    driver.get(url)
    return driver

def reload_page():
    driver = get_driver("http://localhost:5000/")
    driver2 = get_driver("http://localhost:3000/")
    driver3 = get_driver("http://localhost:8000/")
    while True:
        driver.refresh()
        driver2.refresh()
        driver3.refresh()
        time.sleep(0.8)

reload_page()