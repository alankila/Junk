Beating FLAC with libpng
========================

None of the lossless audio compression codecs seem to make use of
autocorrelation when compressing audio.  To put it shortly, autocorrelation is
the property of signal resembling itself when shifted in time.

It is formally defined as:

    sum_i(f(x) * f(x - i))

over some window, where i is the autocorrelation parameter.

Typically, human voice singing or an instrument playing displays extremely high
regularity across some sample window, which depends on the fundamental
frequency of the sound. For instance, if audio is being sampled at 44100 Hz and
contains a fundamental frequency of 440 Hz, then you will find a high
correlation function value near i=100 because 44100 / 440 is approximately 100.

This is because most instrument sounds can be understood as a combination of a
fixed base frequency and overtones that are exact multiples of the base
frequency. Additionally, when multiple sounds are combined, they are often
combined in a fashion that preserves this property, because harmonious sound
require the relationships between base frequencies to be rational numbers based
on fairly small integers. This means that the approach is still viable even
when more complex sounds are being played, though it will work best for human
voice and solo instruments.

What audio compression codecs typically do is use a predictor that is based on
just a few last samples of the sample stream. The difference between the
predicted value and the actual sample value is written as a data stream and
subject to some compression mechanism such as deflate. However, when done this
way the self-similarity of the sample stream is not optimally exploited.

The prototype
-------------

I decided to write a simple prototype to check if I could beat FLAC in its own
game by discovering a good autocorrelation window and using libpng to compress
the same audio as a 16-bit grayscale image. I tested the results using the
PAETH and UP predictors, and did some experimenting with OPTIMUM coding as
well, but I only really got good results by using a single predictor. I settled
with PAETH.

The Paeth algorithm is based on the previous samples in this kind of pattern,
where X is the current sample being coded:

    a b
    c X

In image coding context, a and b are from the prior image row. In audio coding,
a and b are taken from the sample stream behind the autocorrelation length
which serves as "width" of the image. The X is predicted to be one of a, b or c
based on whichever sample is the closest to the Paeth equation b - a + c. The
equation can be understood as b - a measuring the change from a to b, and
adding this to c gives a prediction for what c might be. The fact the
prediction is then replaced by one of the values a, b or c is because images
often reuse the surrounding colors. In audio, such a notion makes no sense, and
should be removed.

As the another alternative, the UP predictor simply takes the sample "b" of the
above schematic and predicts X to be that. In practice this works nearly as well
as Paeth.

Results
-------

The sample audio I chose is one of the best case signals for this kind of
compression, a single gradually decaying piano note. It is instructive to open
the file in image editor to see why UP or PAETH predictor will do a good job on
it. I was able to use libpng to compress the audio to 25k using the best zlib
compression, whereas FLAC -8 compressed it to 27k.

In other words, this half an hour's work beat FLAC in this simple test by about
10 %.

What next?
----------

The current program is mere toy. It will not work for generic signals, music,
speech, or anything like that. It will work for isolated signals like this,
though. To generalize this audio encoding technique, the audio must be analyzed
and divided into suitable chunks based on the varying autocorrelation length,
which would then change from block to block. A stream format must be defined
which can represent the autocorrelation parameter. Better predictor than Paeth
should be chosen, because it is designed for image compression.

It is likely that this kind of mechanism could be retrofitted into Monkey Audio
or FLAC. Only certain kinds of signals benefit from it, so the mode needs to be
enabled and disabled on a frame-by-frame basis.  Because the efficiency of the
coding can be demonstrated to beat existing technologies for some signals, the
output of autocorrelation estimator should be used to determine when to break
signal into a new frame and to indicate the feasibility of using this
prediction mechanism with it.

