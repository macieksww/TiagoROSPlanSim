with open('unprocessed_plan.pddl') as f:
    lines = f.readlines()
    plan_lines = []

    for line in lines:
        if line[0] == "(" and line[1] in [str(0), str(1), str(2), str(3), str(4), str(5), str(6), str(7), str(8), str(9)]:
            plan_lines.append(line)

for line in plan_lines:
    line = line.replace("  ", " ")

    # deletition of lines with actions like "waiting"
    if "->" in line:
        plan_lines.remove(line)

    # conversion to smtplan standard
    # time stamps

splited_lines = []
for line in plan_lines:

    line = line[:line.find(")")] + ":" + line[line.find(")"):]
    line = line.replace(")", "")
    line = line.replace("(", "")
    splited_line = line.split(" ")

    i = 0
    line = ""
    for phrase in splited_line:
        
        if "Parameters:" in phrase:
            del splited_line[i]

        if phrase == "-":
            del splited_line[i]
            del splited_line[i]
        i+=1
    splited_lines.append(splited_line)

plan_lines = []
for splited_line in splited_lines:
    line = ""
    for phrase in splited_line:
        line += phrase + " "
    plan_lines.append(line)

for line in plan_lines:
    line = line.replace("   ", " ")
    line = line[:line.find(":")+1] + " (" + line[line.find(":")+2:]
    line = line[:-3]+")"
    line = line + " [0.0]"
    if line[0] == " ":
        line = line[1:]
    print(line)

with open('plan.pddl', 'w') as f:
    for line in plan_lines:
        f.write(line)

    
    # print(splited_line)