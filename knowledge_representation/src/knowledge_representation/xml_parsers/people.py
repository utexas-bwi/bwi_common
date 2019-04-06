class PersonParser(object):
    def __init__(self):
        #SAVE GENDER AS yaml AND LOAD HERE
        pass
    #XML query for "Where can I find a ({object} | {category})?"
    #returns the defaultlocation or None if item is not an object or category
    def get_age_and_gender(self, pose, person):
        #loop through the poses
            #find match position i to pose
            #if person == "boy":
                #if genders[i] == 'M' and ages[i]  == '0-2' or '4-6' or '8-12':
                    #return True
            #elif person == "girl":
                #if genders[i] == 'F' and ages[i]  == '0-2' or '4-6' or '8-12':
                    #return True
            #elif person == "man" or "male":
                #if genders[i] == 'F' and ages[i]  == '15-20' or '25-32' or '38-43' or '48-53' or '60-100':
                    #return True
            #elif person == "woman" or "female":
                #if genders[i] == 'F' and ages[i]  == '15-20' or '25-32' or '38-43' or '48-53' or '60-100':
                    #return True
        return False
        
    def count_crowd(self, quality):
        #if quality is 'lying':
            #count lying in poses
        #...
        #if quality is 'girl':
            #count = 0
            #for i in range(len(genders)):
                #if genders[i] == 'F' and ages[i] == '0-2' or '4-6' or '8-12':
                    #count = count + 1
            #return count
        #...
        #if quality is 'man':
            #count = 0
            #for i in range(len(genders)):
                #if genders[i] == 'F' and ages[i] == '15-20' or '25-32' or '38-43' or '48-53' or '60-100':
                    #count = count + 1
            #return count
        #...
        #if quality is 'elder':
            #count = 0
            #for i in range(len(ages)):
                #if ages[i] ==  '60-100':
                    #count = count + 1
            #return count
        return 0
            