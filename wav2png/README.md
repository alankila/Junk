Beating FLAC with libpng
========================

None of the audio compression codecs seem to make use of autocorrelation when
compressing audio.  To put it shortly, autocorrelation is the property of
signal resembling itself when shifted in time.

It is formally defined as:

    sum_i(f(x) * f(x - i))

over some window, where i is the autocorrelation parameter.

Typically, human voice singing or an instrument playing displays extremely high
regularity across some sample window, which depends on the fundamental
frequency of the sound. For instance, if audio is being sampled at 44100 Hz and
contains a fundamental frequency of 440 Hz, then you will find a high
correlation function value near i=100 because 44100 / 440 is approximately 100.

Results
-------

I decided to write a simple prototype to check if I could beat FLAC in its own
game by discovering a good autocorrelation window and using libpng to compress
the same audio as a 16-bit grayscale image. I tested the results using the
PAETH and UP predictors, and did some experimenting with OPTIMUM coding as
well, but I only really got good results by using a single predictor. I settled
with PAETH.

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
speech, or anything like that. It will work for single signals like this,
though. To generalize this audio encoding technique, the audio must be analyzed
and divided into suitable chunks based on the varying autocorrelation length,
which would then change from block to block. A stream format must be defined
which can represent the autocorrelation parameter. More complicated prediction
functions could be chosen than PAETH which is really designed for image
compression.

