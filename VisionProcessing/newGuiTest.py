import tkinter as tk
import tkinter.ttk as ttk

app = tk.Tk()
#app.geometry('800x600')

X = 600
Y = 600
GRID = 15

c = tk.Canvas(app, width=X+1, height=Y+1, highlightthickness=0, bg='#aaa')
c.grid(row=0, column=0, padx=15, pady=15)

ov_size=(6,6)

def get_oval_centre(coords):
    x = (coords[0] + coords[2])//2
    y = (coords[1] + coords[3])//2
    return [x,y]

def set_oval_coords(t, xy):
    x,y = xy
    c.coords(t, (x-5, y-5, x+5, y+5) )

def evn(e):
    xx = c.canvasx(e.x)
    yy = c.canvasy(e.y)
    #cc = c.find_closest(xx,yy)
    t = c.find_withtag('point')
    #c.move(t, 20,20)
    #print(c.find_all())
    #c.itemconfig(t, )
    oc = get_oval_centre(c.bbox(t))
    oc[0] += GRID
    oc[1] += GRID

def mot(e):
    xg = (e.x+GRID/2)//GRID
    yg = (e.y+GRID/2)//GRID
    t = c.find_withtag('point')
    set_oval_coords(t, (xg*GRID, yg*GRID))

c.bind('<Button-1>', evn)
c.bind('<Motion>', mot)


for i in range(0,Y, GRID):
    c.create_line(0,i, X,i, fill='#999')
for i in range(0,X, GRID):
    c.create_line(i,0, i,Y, fill='#999')

ov = c.create_oval(-3,-3, 3, 3, tags='point', fill='red')

b = ttk.Button(app, text='quit', command=app.destroy)
b.grid(row=2, column=0, padx=10, pady=15)


app.mainloop()
