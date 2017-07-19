# balanc3r-for-ev3way

Robot balancing program for EV3way-ET. This program made by Python.



# Prerequisite

- Body: [EV3Way-ET](https://github.com/ETrobocon/etroboEV3/wiki)

- OS: [ev3dev](http://www.ev3dev.org/)




# How to use

In host shell,  login your EV3Way-ET by ssh.

```shell
$ ssh username@xxx.xxx.xxx.xxx
```

In EV3(ev3dev) shell, Execute "ps lax" to find processes given "-20" nice value.

```shell
$ ps lax | grep -- [-]20

  UID   PID  PPID CPU PRI NI  VSZ       RSS  WCHAN  STAT   TT    TIME    COMMAND
  0     120  0    0   31  -20 2480868   8768 -      Ss     ??    2:32.12 hogehoge
```

And execute "renice" for all displayed processes to give "0" nice value.

```shell
$ sudo renice 0 -p 120
```

Then, Get this repository and execute "stop_deamons.sh".

```shell
$ git clone https://github.com/ETRobocon2017-TeamD/balanc3r-for-ev3way-et.git
$ cd balanc3r-for-ev3way-et
$ sudo chmod 577 stop_daemons.sh
$ ./stop_daemons.sh
```

After, Execute "pistol.py" by "nice" command to give "-12" nice value.

```shell
$ sudo nice -n -12 python3 pistol.py
```


# Reference

- [SC4050+ Integration Project: Balancing Robot](http://laurensvalk.com/files/Bos_Valk_SC4050_Balancing_Robot.pdf)
- [NXTWay-GS(Self-Balancing Tow-Wheeled Robot) Controller Design](http://jp.mathworks.com/matlabcentral/fileexchange/19147-nxtway-gs--self-balancing-two-wheeled-robot--controller-design)
- [ev3dev issue+ Tips to get a more constant loop time](https://github.com/ev3dev/ev3dev/issues/324)
- [segway](https://github.com/laurensvalk/segway)
- [ETrobocon/etroboEV3 (in Japanese)](https://github.com/ETrobocon/etroboEV3)




# Acknowledgements

We are grateful to [The ETrobocon 2017 executive committee](http://www.etrobo.jp/).

We want to thank [segway](https://github.com/laurensvalk/segway) author [Laurens Valk](http://laurensvalk.com/), and [ev3dev-lang-python](https://github.com/rhempel/ev3dev-lang-python) developer [Ralph Hempel](https://github.com/rhempel) and commiters.

