
a = raw_input('file name : ')
filename = a + '.txt'
letter = open(filename, 'a+')
letter.write('\n\nHow are you?')
letter.close()


