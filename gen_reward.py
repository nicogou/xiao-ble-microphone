from PIL import Image
import os

SIZE = 120

img = Image.open('music.png').convert('RGBA').resize((SIZE, SIZE), Image.LANCZOS)

img.save('reward_preview.png')
print('Saved reward_preview.png')

# LVGL 9 RGB565A8: all RGB565 pixels first, then all alpha values (separate planes).
rgb_data = []
alpha_data = []
for r, g, b, a in img.getdata():
    rgb = ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3)
    rgb_data += [rgb & 0xFF, (rgb >> 8) & 0xFF]
    alpha_data.append(a)

data = rgb_data + alpha_data

out  = ['#pragma once', '#include <lvgl.h>', '']
out += ['static const uint8_t reward_img_map[] = {']
for i in range(0, len(data), 12):
    out.append('    ' + ', '.join('0x{:02X}'.format(b) for b in data[i:i+12]) + ',')
out += ['};', '',
        'static const lv_image_dsc_t reward_img = {',
        '    .header = {',
        '        .magic      = LV_IMAGE_HEADER_MAGIC,',
        '        .cf         = LV_COLOR_FORMAT_NATIVE_WITH_ALPHA,',
        '        .flags      = 0,',
        '        .w          = ' + str(SIZE) + ',',
        '        .h          = ' + str(SIZE) + ',',
        '        .stride     = ' + str(SIZE * 2) + ',',
        '        .reserved_2 = 0,',
        '    },',
        '    .data_size = ' + str(len(data)) + ',',
        '    .data      = reward_img_map,',
        '};']

path = os.path.join('app', 'src', 'reward_img.h')
with open(path, 'w') as f:
    f.write('\n'.join(out) + '\n')
print('OK: ' + path + '  (' + str(len(data)) + ' bytes, ' + str(SIZE) + 'x' + str(SIZE) + ' px)')
