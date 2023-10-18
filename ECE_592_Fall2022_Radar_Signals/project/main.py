#!/usr/local/bin/python3


from tkinter import *
from tkinter import ttk



if __name__=="__main__":
  main = Tk()
  frame=ttk.Frame(main, padding=100)
  frame.grid()
  ttk.Label(frame, text="Hellow World").grid(column=0, row=0)
  ttk.Label(frame, text="Target Soeed").grid(column=1, row=0)
  ttk.Label(frame, text="Target Strength").grid(column=1, row=1)
  ttk.Label(frame, text="[Doppler Frequency]").grid(column=1, row=2)

  ttk.Button(frame, text="Push me", command=main.destroy).grid(column=0, row=1)
  main.title("Arnie's Toy Radar Applicaiton")
  #main.geometry( "600x400" )
  main.mainloop()