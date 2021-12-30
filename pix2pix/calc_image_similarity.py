import cv2
# import imagehash
from PIL import Image
import numpy as np
from scipy.spatial import distance

"""
def get_hash(img_path):
    try:
        hash_val = imagehash.phash(Image.open(img_path))
    except cv2.error:
        return "error"
    return hash
"""

def hash_array_to_hash_hex(hash_array):
  # convert hash array of 0 or 1 to hash string in hex
  hash_array = np.array(hash_array, dtype = np.uint8)
  hash_str = ''.join(str(i) for i in 1 * hash_array.flatten())
  return (hex(int(hash_str, 2)))

def hash_hex_to_hash_array(hash_hex):
  # convert hash string in hex to hash values of 0 or 1
  hash_str = int(hash_hex, 16)
  array_str = bin(hash_str)[2:]
  return np.array([i for i in array_str], dtype = np.float32)

def hash_main(img_path):
  img = cv2.imread(img_path)
  # resize image and convert to gray scale
  img = cv2.resize(img, (64, 64))
  img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
  img = np.array(img, dtype = np.float32)
  # calculate dct of image 
  dct = cv2.dct(img)
  # to reduce hash length take only 8*8 top-left block 
  # as this block has more information than the rest
  dct_block = dct[: 8, : 8]
  # caclulate mean of dct block excluding first term i.e, dct(0, 0)
  dct_average = (dct_block.mean() * dct_block.size - dct_block[0, 0]) / (dct_block.size - 1)
  # convert dct block to binary values based on dct_average
  dct_block[dct_block < dct_average] = 0.0
  dct_block[dct_block != 0] = 1.0

  return hash_array_to_hash_hex(dct_block.flatten())


hash0 = hash_main("share/sim_0.png")
hash1 = hash_main("share/sim_1.png")
hash2 = hash_main("share/sim_2.png")
print(hash0, hash1, hash2)

dis1 = distance.hamming(list(hash0), list(hash1))
dis2 = distance.hamming(list(hash0), list(hash2))
print(dis1, dis2)

"""
dis1 = distance.hamming(hash_val0, hash_val1)
dis2 = distance.hamming(hash_val1, hash_val2)
print(dis1, dis2)
"""

# https://ichi.pro/python-no-gazo-hasshu-gijutsu-o-shiyoshita-gazo-kensaku-enjin-253388535941367
