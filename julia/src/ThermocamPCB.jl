module ThermocamPCB

using Gnuplot
using ColorTypes
using Colors
using ColorVectorSpace
using TiffImages
using Statistics
using Pipe
using ImageTransformations, Interpolations

export load, immax, imdiff, immix, resize

struct Img
    title::String
    data::Matrix{Float64}
end

function load(input::Tuple{String, String})
    filename, title = input
    Img(title, @pipe TiffImages.load(filename).data |> map(_) do x x.val end)
end

"Load tiff image produced by thermocam-pcb web server"
function load(filename::String)
    (title, ext) = splitext(filename)
    for strip_prefix in ["lapgz-avg1-", "lapgz-mean-"]
        if startswith(title, strip_prefix)
            title = title[length(strip_prefix)+1:end]
        end
    end
    load((filename, title))
end

resize(im::Img, ratio=3) = Img(im.title, imresize(im.data, ratio=ratio))

color(i::Int64, len::Int64, value, sat=1) = HSV.(i*360.0/len, sat, value)

function legend(imgs)
    rgb(i) = hex(convert(RGB, color(i, length(imgs), 1)))
    [ Gnuplot.PlotElement(cmds=["set key horizontal reverse Left left textcolor rgb('white')"]),
      #Gnuplot.PlotElement(cmds=["set key outside reverse Left right opaque"]),
      [Gnuplot.PlotElement(data=Gnuplot.DatasetText(["NaN"]),
                           plot="w p pt 5 lc rgb(0x$(rgb(i))) title '$(imgs[i].title)'") for i in 1:length(imgs)]...
    ]
end

function colorbar(max; valxform=identity)
    palette_steps = ["$x $(valxform(x)/valxform(1)) $(valxform(x)/valxform(1)) $(valxform(x)/valxform(1))" for x in 0:0.05:1]
    Gnuplot.PlotElement(cmds=["set palette defined ($(join(palette_steps, ',')))", "set cbrange [0:$max]"], plot="with image notitle", data=Gnuplot.DatasetText([0,0,0]))
end

"""
Show which heatmap dominates others. The color of each pixel is
calculated as follows:
- intensity: *maximum* heat value among all heatmaps
- saturation: the difference between first and seconds highest heat value
- hue: heatmap index
"""
function immax(heatmaps::Vector{Img})
    imgsdata = [im.data for im in heatmaps]
    mx = maximum(max.(imgsdata...))
    diff_to_others = map(eachindex(heatmaps)) do i
        max_other = max.(imgsdata[filter(!=(i), eachindex(heatmaps))]...)
        d = imgsdata[i] - max_other
    end
    mxdiff = maximum(max.(diff_to_others...))
    finalimg = sum(eachindex(heatmaps)) do i
        dd = imgsdata[i] .* (diff_to_others[i] .> 0)
        convert.(RGB, color.(i, length(heatmaps), dd/mx, diff_to_others[i]/mxdiff))
    end
    @gp "unset xtics" "unset ytics" finalimg legend(heatmaps) colorbar(mx)
end

"Function that can be used as valxform for `imdiff`"
mylog(x) = let c=0.005; log(x+c)-log(c) end

"Plot the difference the first and second highest heat source. Hue is
the source, intensity is the difference."
function imdiff(imgs; valxform=mylog)
    imgsdata = [im.data for im in imgs]
    function maxdiff(imgsdata, i)
        indices = [collect(1:i-1); collect(i+1:length(imgsdata))]
        max_other = max.(imgsdata[indices]...)
        d = imgsdata[i] - max_other
        d .* (d .> 0)
    end
    dds = [maxdiff(imgsdata, i) for i in 1:length(imgsdata)]
    mx = maximum(valxform.(max.(dds...)))
    finalimg = sum(1:length(imgsdata)) do i
        convert.(RGB, color.(i, length(imgs), valxform.(dds[i])/mx))
    end
    @gp title="Dominating heat source" "unset xtics" "unset ytics" :-
    @gp :- 1 finalimg legend(imgs) colorbar(maximum(maximum.(dds)), valxform=valxform)
end

function immix(imgsdata::Vector{Matrix{Float64}})
    mx = maximum(max.(imgsdata...))
    finalimg = sum(1:length(imgsdata)) do i
        convert.(RGB, color.(i, length(imgsdata), imgsdata[i]/mx))
    end
    ## Normalize again if the colors sum to value greater > 1. Can
    ## happen if we have more than 3 imgs, i.e., more colors than
    ## R, G, and B.
    mx = map(finalimg) do x max(x.r, x.g, x.b) end |> maximum
    finalimg./mx
end
function immix(imgs::Vector{Img})
    data = [im.data for im in imgs]
    @gp "unset xtics" "unset ytics" immix(data) legend(imgs) colorbar(maximum(maximum.(data)))
end

end
