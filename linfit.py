import numpy as np
import matplotlib.pyplot as plt

x = [887, 386, 648, 961]
y = [866, 349, 608, 923]


fit = np.polyfit(x,y,1)
fit_fn = np.poly1d(fit)

print '%.6f,' % fit[0], '%.6f' % fit[1]
print 'Residues [cm]:'
r = fit_fn(x) - y
print ["%.1f" % val for val in r]

plt.plot(x,y, 'ro', x, fit_fn(x), '-k')
plt.xlim(0, 1200)
plt.ylim(0, 1200)
plt.grid()
plt.show()

