/*
 *
 * Copyright (c) 2023 Texas Instruments Incorporated
 *
 * All rights reserved not granted herein.
 *
 * Limited License.
 *
 * Texas Instruments Incorporated grants a world-wide, royalty-free, non-exclusive
 * license under copyrights and patents it now or hereafter owns or controls to make,
 * have made, use, import, offer to sell and sell ("Utilize") this software subject to the
 * terms herein.  With respect to the foregoing patent license, such license is granted
 * solely to the extent that any such patent is necessary to Utilize the software alone.
 * The patent license shall not apply to any combinations which include this software,
 * other than combinations with devices manufactured by or for TI ("TI Devices").
 * No hardware patent is licensed hereunder.
 *
 * Redistributions must preserve existing copyright notices and reproduce this license
 * (including the above copyright notice and the disclaimer and (if applicable) source
 * code license limitations below) in the documentation and/or other materials provided
 * with the distribution
 *
 * Redistribution and use in binary form, without modification, are permitted provided
 * that the following conditions are met:
 *
 * *       No reverse engineering, decompilation, or disassembly of this software is
 * permitted with respect to any software provided in binary form.
 *
 * *       any redistribution and use are licensed by TI for use only with TI Devices.
 *
 * *       Nothing shall obligate TI to provide you with source code for the software
 * licensed and provided to you in object code.
 *
 * If software source code is provided to you, modification and redistribution of the
 * source code are permitted provided that the following conditions are met:
 *
 * *       any redistribution and use of the source code, including any resulting derivative
 * works, are licensed by TI for use only with TI Devices.
 *
 * *       any redistribution and use of any object code compiled from the source code
 * and any resulting derivative works, are licensed by TI for use only with TI Devices.
 *
 * Neither the name of Texas Instruments Incorporated nor the names of its suppliers
 *
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * DISCLAIMER.
 *
 * THIS SOFTWARE IS PROVIDED BY TI AND TI'S LICENSORS "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL TI AND TI'S LICENSORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 * OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <tiovx_utils.h>
#include "tiovx_dl_pre_proc_module.h"
#include "tiovx_tidl_module.h"
#include "tiovx_dl_post_proc_module.h"
#include <TI/dl_kernels.h>

#define APP_BUFQ_DEPTH (1)

#define IMAGE_WIDTH (224)
#define IMAGE_HEIGHT (224)

#define TIDL_CONFIG_FILE_PATH "/opt/vision_apps/test_data/tivx/tidl_models/tidl_io_mobilenet_v1_1.bin"
#define TIDL_NETWORK_FILE_PATH "/opt/vision_apps/test_data/tivx/tidl_models/tidl_net_mobilenet_v1.bin"

const char imgnet_labels[1001][256] =
{
  "background",
  "tench, Tinca tinca",
  "goldfish, Carassius auratus",
  "great white shark, white shark, man-eater, man-eating shark, Carcharodon carcharias",
  "tiger shark, Galeocerdo cuvieri",
  "hammerhead, hammerhead shark",
  "electric ray, crampfish, numbfish, torpedo",
  "stingray",
  "cock",
  "hen",
  "ostrich, Struthio camelus",
  "brambling, Fringilla montifringilla",
  "goldfinch, Carduelis carduelis",
  "house finch, linnet, Carpodacus mexicanus",
  "junco, snowbird",
  "indigo bunting, indigo finch, indigo bird, Passerina cyanea",
  "robin, American robin, Turdus migratorius",
  "bulbul",
  "jay",
  "magpie",
  "chickadee",
  "water ouzel, dipper",
  "kite",
  "bald eagle, American eagle, Haliaeetus leucocephalus",
  "vulture",
  "great grey owl, great gray owl, Strix nebulosa",
  "European fire salamander, Salamandra salamandra",
  "common newt, Triturus vulgaris",
  "eft",
  "spotted salamander, Ambystoma maculatum",
  "axolotl, mud puppy, Ambystoma mexicanum",
  "bullfrog, Rana catesbeiana",
  "tree frog, tree-frog",
  "tailed frog, bell toad, ribbed toad, tailed toad, Ascaphus trui",
  "loggerhead, loggerhead turtle, Caretta caretta",
  "leatherback turtle, leatherback, leathery turtle, Dermochelys coriacea",
  "mud turtle",
  "terrapin",
  "box turtle, box tortoise",
  "banded gecko",
  "common iguana, iguana, Iguana iguana",
  "American chameleon, anole, Anolis carolinensis",
  "whiptail, whiptail lizard",
  "agama",
  "frilled lizard, Chlamydosaurus kingi",
  "alligator lizard",
  "Gila monster, Heloderma suspectum",
  "green lizard, Lacerta viridis",
  "African chameleon, Chamaeleo chamaeleon",
  "Komodo dragon, Komodo lizard, dragon lizard, giant lizard, Varanus komodoensis",
  "African crocodile, Nile crocodile, Crocodylus niloticus",
  "American alligator, Alligator mississipiensis",
  "triceratops",
  "thunder snake, worm snake, Carphophis amoenus",
  "ringneck snake, ring-necked snake, ring snake",
  "hognose snake, puff adder, sand viper",
  "green snake, grass snake",
  "king snake, kingsnake",
  "garter snake, grass snake",
  "water snake",
  "vine snake",
  "night snake, Hypsiglena torquata",
  "boa constrictor, Constrictor constrictor",
  "rock python, rock snake, Python sebae",
  "Indian cobra, Naja naja",
  "green mamba",
  "sea snake",
  "horned viper, cerastes, sand viper, horned asp, Cerastes cornutus",
  "diamondback, diamondback rattlesnake, Crotalus adamanteus",
  "sidewinder, horned rattlesnake, Crotalus cerastes",
  "trilobite",
  "harvestman, daddy longlegs, Phalangium opilio",
  "scorpion",
  "black and gold garden spider, Argiope aurantia",
  "barn spider, Araneus cavaticus",
  "garden spider, Aranea diademata",
  "black widow, Latrodectus mactans",
  "tarantula",
  "wolf spider, hunting spider",
  "tick",
  "centipede",
  "black grouse",
  "ptarmigan",
  "ruffed grouse, partridge, Bonasa umbellus",
  "prairie chicken, prairie grouse, prairie fowl",
  "peacock",
  "quail",
  "partridge",
  "African grey, African gray, Psittacus erithacus",
  "macaw",
  "sulphur-crested cockatoo, Kakatoe galerita, Cacatua galerita",
  "lorikeet",
  "coucal",
  "bee eater",
  "hornbill",
  "hummingbird",
  "jacamar",
  "toucan",
  "drake",
  "red-breasted merganser, Mergus serrator",
  "goose",
  "black swan, Cygnus atratus",
  "tusker",
  "echidna, spiny anteater, anteater",
  "platypus, duckbill, duckbilled platypus, duck-billed platypus, Ornithorhynchus anatinus",
  "wallaby, brush kangaroo",
  "koala, koala bear, kangaroo bear, native bear, Phascolarctos cinereus",
  "wombat",
  "jellyfish",
  "sea anemone, anemone",
  "brain coral",
  "flatworm, platyhelminth",
  "nematode, nematode worm, roundworm",
  "conch",
  "snail",
  "slug",
  "sea slug, nudibranch",
  "chiton, coat-of-mail shell, sea cradle, polyplacophore",
  "chambered nautilus, pearly nautilus, nautilus",
  "Dungeness crab, Cancer magister",
  "rock crab, Cancer irroratus",
  "fiddler crab",
  "king crab, Alaska crab, Alaskan king crab, Alaska king crab, Paralithodes camtschatica",
  "American lobster, Northern lobster, Maine lobster, Homarus americanus",
  "spiny lobster, langouste, rock lobster, crawfish, crayfish, sea crawfish",
  "crayfish, crawfish, crawdad, crawdaddy",
  "hermit crab",
  "isopod",
  "white stork, Ciconia ciconia",
  "black stork, Ciconia nigra",
  "spoonbill",
  "flamingo",
  "little blue heron, Egretta caerulea",
  "American egret, great white heron, Egretta albus",
  "bittern",
  "crane",
  "limpkin, Aramus pictus",
  "European gallinule, Porphyrio porphyrio",
  "American coot, marsh hen, mud hen, water hen, Fulica americana",
  "bustard",
  "ruddy turnstone, Arenaria interpres",
  "red-backed sandpiper, dunlin, Erolia alpina",
  "redshank, Tringa totanus",
  "dowitcher",
  "oystercatcher, oyster catcher",
  "pelican",
  "king penguin, Aptenodytes patagonica",
  "albatross, mollymawk",
  "grey whale, gray whale, devilfish, Eschrichtius gibbosus, Eschrichtius robustus",
  "killer whale, killer, orca, grampus, sea wolf, Orcinus orca",
  "dugong, Dugong dugon",
  "sea lion",
  "Chihuahua",
  "Japanese spaniel",
  "Maltese dog, Maltese terrier, Maltese",
  "Pekinese, Pekingese, Peke",
  "Shih-Tzu",
  "Blenheim spaniel",
  "papillon",
  "toy terrier",
  "Rhodesian ridgeback",
  "Afghan hound, Afghan",
  "basset, basset hound",
  "beagle",
  "bloodhound, sleuthhound",
  "bluetick",
  "black-and-tan coonhound",
  "Walker hound, Walker foxhound",
  "English foxhound",
  "redbone",
  "borzoi, Russian wolfhound",
  "Irish wolfhound",
  "Italian greyhound",
  "whippet",
  "Ibizan hound, Ibizan Podenco",
  "Norwegian elkhound, elkhound",
  "otterhound, otter hound",
  "Saluki, gazelle hound",
  "Scottish deerhound, deerhound",
  "Weimaraner",
  "Staffordshire bullterrier, Staffordshire bull terrier",
  "American Staffordshire terrier, Staffordshire terrier, American pit bull terrier, pit bull terrier",
  "Bedlington terrier",
  "Border terrier",
  "Kerry blue terrier",
  "Irish terrier",
  "Norfolk terrier",
  "Norwich terrier",
  "Yorkshire terrier",
  "wire-haired fox terrier",
  "Lakeland terrier",
  "Sealyham terrier, Sealyham",
  "Airedale, Airedale terrier",
  "cairn, cairn terrier",
  "Australian terrier",
  "Dandie Dinmont, Dandie Dinmont terrier",
  "Boston bull, Boston terrier",
  "miniature schnauzer",
  "giant schnauzer",
  "standard schnauzer",
  "Scotch terrier, Scottish terrier, Scottie",
  "Tibetan terrier, chrysanthemum dog",
  "silky terrier, Sydney silky",
  "soft-coated wheaten terrier",
  "West Highland white terrier",
  "Lhasa, Lhasa apso",
  "flat-coated retriever",
  "curly-coated retriever",
  "golden retriever",
  "Labrador retriever",
  "Chesapeake Bay retriever",
  "German short-haired pointer",
  "vizsla, Hungarian pointer",
  "English setter",
  "Irish setter, red setter",
  "Gordon setter",
  "Brittany spaniel",
  "clumber, clumber spaniel",
  "English springer, English springer spaniel",
  "Welsh springer spaniel",
  "cocker spaniel, English cocker spaniel, cocker",
  "Sussex spaniel",
  "Irish water spaniel",
  "kuvasz",
  "schipperke",
  "groenendael",
  "malinois",
  "briard",
  "kelpie",
  "komondor",
  "Old English sheepdog, bobtail",
  "Shetland sheepdog, Shetland sheep dog, Shetland",
  "collie",
  "Border collie",
  "Bouvier des Flandres, Bouviers des Flandres",
  "Rottweiler",
  "German shepherd, German shepherd dog, German police dog, alsatian",
  "Doberman, Doberman pinscher",
  "miniature pinscher",
  "Greater Swiss Mountain dog",
  "Bernese mountain dog",
  "Appenzeller",
  "EntleBucher",
  "boxer",
  "bull mastiff",
  "Tibetan mastiff",
  "French bulldog",
  "Great Dane",
  "Saint Bernard, St Bernard",
  "Eskimo dog, husky",
  "malamute, malemute, Alaskan malamute",
  "Siberian husky",
  "dalmatian, coach dog, carriage dog",
  "affenpinscher, monkey pinscher, monkey dog",
  "basenji",
  "pug, pug-dog",
  "Leonberg",
  "Newfoundland, Newfoundland dog",
  "Great Pyrenees",
  "Samoyed, Samoyede",
  "Pomeranian",
  "chow, chow chow",
  "keeshond",
  "Brabancon griffon",
  "Pembroke, Pembroke Welsh corgi",
  "Cardigan, Cardigan Welsh corgi",
  "toy poodle",
  "miniature poodle",
  "standard poodle",
  "Mexican hairless",
  "timber wolf, grey wolf, gray wolf, Canis lupus",
  "white wolf, Arctic wolf, Canis lupus tundrarum",
  "red wolf, maned wolf, Canis rufus, Canis niger",
  "coyote, prairie wolf, brush wolf, Canis latrans",
  "dingo, warrigal, warragal, Canis dingo",
  "dhole, Cuon alpinus",
  "African hunting dog, hyena dog, Cape hunting dog, Lycaon pictus",
  "hyena, hyaena",
  "red fox, Vulpes vulpes",
  "kit fox, Vulpes macrotis",
  "Arctic fox, white fox, Alopex lagopus",
  "grey fox, gray fox, Urocyon cinereoargenteus",
  "tabby, tabby cat",
  "tiger cat",
  "Persian cat",
  "Siamese cat, Siamese",
  "Egyptian cat",
  "cougar, puma, catamount, mountain lion, painter, panther, Felis concolor",
  "lynx, catamount",
  "leopard, Panthera pardus",
  "snow leopard, ounce, Panthera uncia",
  "jaguar, panther, Panthera onca, Felis onca",
  "lion, king of beasts, Panthera leo",
  "tiger, Panthera tigris",
  "cheetah, chetah, Acinonyx jubatus",
  "brown bear, bruin, Ursus arctos",
  "American black bear, black bear, Ursus americanus, Euarctos americanus",
  "ice bear, polar bear, Ursus Maritimus, Thalarctos maritimus",
  "sloth bear, Melursus ursinus, Ursus ursinus",
  "mongoose",
  "meerkat, mierkat",
  "tiger beetle",
  "ladybug, ladybeetle, lady beetle, ladybird, ladybird beetle",
  "ground beetle, carabid beetle",
  "long-horned beetle, longicorn, longicorn beetle",
  "leaf beetle, chrysomelid",
  "dung beetle",
  "rhinoceros beetle",
  "weevil",
  "fly",
  "bee",
  "ant, emmet, pismire",
  "grasshopper, hopper",
  "cricket",
  "walking stick, walkingstick, stick insect",
  "cockroach, roach",
  "mantis, mantid",
  "cicada, cicala",
  "leafhopper",
  "lacewing, lacewing fly",
  "dragonfly, darning needle, devil's darning needle, sewing needle, snake feeder, snake doctor, mosquito hawk, skeeter hawk",
  "damselfly",
  "admiral",
  "ringlet, ringlet butterfly",
  "monarch, monarch butterfly, milkweed butterfly, Danaus plexippus",
  "cabbage butterfly",
  "sulphur butterfly, sulfur butterfly",
  "lycaenid, lycaenid butterfly",
  "starfish, sea star",
  "sea urchin",
  "sea cucumber, holothurian",
  "wood rabbit, cottontail, cottontail rabbit",
  "hare",
  "Angora, Angora rabbit",
  "hamster",
  "porcupine, hedgehog",
  "fox squirrel, eastern fox squirrel, Sciurus niger",
  "marmot",
  "beaver",
  "guinea pig, Cavia cobaya",
  "sorrel",
  "zebra",
  "hog, pig, grunter, squealer, Sus scrofa",
  "wild boar, boar, Sus scrofa",
  "warthog",
  "hippopotamus, hippo, river horse, Hippopotamus amphibius",
  "ox",
  "water buffalo, water ox, Asiatic buffalo, Bubalus bubalis",
  "bison",
  "ram, tup",
  "bighorn, bighorn sheep, cimarron, Rocky Mountain bighorn, Rocky Mountain sheep, Ovis canadensis",
  "ibex, Capra ibex",
  "hartebeest",
  "impala, Aepyceros melampus",
  "gazelle",
  "Arabian camel, dromedary, Camelus dromedarius",
  "llama",
  "weasel",
  "mink",
  "polecat, fitch, foulmart, foumart, Mustela putorius",
  "black-footed ferret, ferret, Mustela nigripes",
  "otter",
  "skunk, polecat, wood pussy",
  "badger",
  "armadillo",
  "three-toed sloth, ai, Bradypus tridactylus",
  "orangutan, orang, orangutang, Pongo pygmaeus",
  "gorilla, Gorilla gorilla",
  "chimpanzee, chimp, Pan troglodytes",
  "gibbon, Hylobates lar",
  "siamang, Hylobates syndactylus, Symphalangus syndactylus",
  "guenon, guenon monkey",
  "patas, hussar monkey, Erythrocebus patas",
  "baboon",
  "macaque",
  "langur",
  "colobus, colobus monkey",
  "proboscis monkey, Nasalis larvatus",
  "marmoset",
  "capuchin, ringtail, Cebus capucinus",
  "howler monkey, howler",
  "titi, titi monkey",
  "spider monkey, Ateles geoffroyi",
  "squirrel monkey, Saimiri sciureus",
  "Madagascar cat, ring-tailed lemur, Lemur catta",
  "indri, indris, Indri indri, Indri brevicaudatus",
  "Indian elephant, Elephas maximus",
  "African elephant, Loxodonta africana",
  "lesser panda, red panda, panda, bear cat, cat bear, Ailurus fulgens",
  "giant panda, panda, panda bear, coon bear, Ailuropoda melanoleuca",
  "barracouta, snoek",
  "eel",
  "coho, cohoe, coho salmon, blue jack, silver salmon, Oncorhynchus kisutch",
  "rock beauty, Holocanthus tricolor",
  "anemone fish",
  "sturgeon",
  "gar, garfish, garpike, billfish, Lepisosteus osseus",
  "lionfish",
  "puffer, pufferfish, blowfish, globefish",
  "abacus",
  "abaya",
  "academic gown, academic robe, judge's robe",
  "accordion, piano accordion, squeeze box",
  "acoustic guitar",
  "aircraft carrier, carrier, flattop, attack aircraft carrier",
  "airliner",
  "airship, dirigible",
  "altar",
  "ambulance",
  "amphibian, amphibious vehicle",
  "analog clock",
  "apiary, bee house",
  "apron",
  "ashcan, trash can, garbage can, wastebin, ash bin, ash-bin, ashbin, dustbin, trash barrel, trash bin",
  "assault rifle, assault gun",
  "backpack, back pack, knapsack, packsack, rucksack, haversack",
  "bakery, bakeshop, bakehouse",
  "balance beam, beam",
  "balloon",
  "ballpoint, ballpoint pen, ballpen, Biro",
  "Band Aid",
  "banjo",
  "bannister, banister, balustrade, balusters, handrail",
  "barbell",
  "barber chair",
  "barbershop",
  "barn",
  "barometer",
  "barrel, cask",
  "barrow, garden cart, lawn cart, wheelbarrow",
  "baseball",
  "basketball",
  "bassinet",
  "bassoon",
  "bathing cap, swimming cap",
  "bath towel",
  "bathtub, bathing tub, bath, tub",
  "beach wagon, station wagon, wagon, estate car, beach waggon, station waggon, waggon",
  "beacon, lighthouse, beacon light, pharos",
  "beaker",
  "bearskin, busby, shako",
  "beer bottle",
  "beer glass",
  "bell cote, bell cot",
  "bib",
  "bicycle-built-for-two, tandem bicycle, tandem",
  "bikini, two-piece",
  "binder, ring-binder",
  "binoculars, field glasses, opera glasses",
  "birdhouse",
  "boathouse",
  "bobsled, bobsleigh, bob",
  "bolo tie, bolo, bola tie, bola",
  "bonnet, poke bonnet",
  "bookcase",
  "bookshop, bookstore, bookstall",
  "bottlecap",
  "bow",
  "bow tie, bow-tie, bowtie",
  "brass, memorial tablet, plaque",
  "brassiere, bra, bandeau",
  "breakwater, groin, groyne, mole, bulwark, seawall, jetty",
  "breastplate, aegis, egis",
  "broom",
  "bucket, pail",
  "buckle",
  "bulletproof vest",
  "bullet train, bullet",
  "butcher shop, meat market",
  "cab, hack, taxi, taxicab",
  "caldron, cauldron",
  "candle, taper, wax light",
  "cannon",
  "canoe",
  "can opener, tin opener",
  "cardigan",
  "car mirror",
  "carousel, carrousel, merry-go-round, roundabout, whirligig",
  "carpenter's kit, tool kit",
  "carton",
  "car wheel",
  "cash machine, cash dispenser, automated teller machine, automatic teller machine, automated teller, automatic teller, ATM",
  "cassette",
  "cassette player",
  "castle",
  "catamaran",
  "CD player",
  "cello, violoncello",
  "cellular telephone, cellular phone, cellphone, cell, mobile phone",
  "chain",
  "chainlink fence",
  "chain mail, ring mail, mail, chain armor, chain armour, ring armor, ring armour",
  "chain saw, chainsaw",
  "chest",
  "chiffonier, commode",
  "chime, bell, gong",
  "china cabinet, china closet",
  "Christmas stocking",
  "church, church building",
  "cinema, movie theater, movie theatre, movie house, picture palace",
  "cleaver, meat cleaver, chopper",
  "cliff dwelling",
  "cloak",
  "clog, geta, patten, sabot",
  "cocktail shaker",
  "coffee mug",
  "coffeepot",
  "coil, spiral, volute, whorl, helix",
  "combination lock",
  "computer keyboard, keypad",
  "confectionery, confectionary, candy store",
  "container ship, containership, container vessel",
  "convertible",
  "corkscrew, bottle screw",
  "cornet, horn, trumpet, trump",
  "cowboy boot",
  "cowboy hat, ten-gallon hat",
  "cradle",
  "crane",
  "crash helmet",
  "crate",
  "crib, cot",
  "Crock Pot",
  "croquet ball",
  "crutch",
  "cuirass",
  "dam, dike, dyke",
  "desk",
  "desktop computer",
  "dial telephone, dial phone",
  "diaper, nappy, napkin",
  "digital clock",
  "digital watch",
  "dining table, board",
  "dishrag, dishcloth",
  "dishwasher, dish washer, dishwashing machine",
  "disk brake, disc brake",
  "dock, dockage, docking facility",
  "dogsled, dog sled, dog sleigh",
  "dome",
  "doormat, welcome mat",
  "drilling platform, offshore rig",
  "drum, membranophone, tympan",
  "drumstick",
  "dumbbell",
  "Dutch oven",
  "electric fan, blower",
  "electric guitar",
  "electric locomotive",
  "entertainment center",
  "envelope",
  "espresso maker",
  "face powder",
  "feather boa, boa",
  "file, file cabinet, filing cabinet",
  "fireboat",
  "fire engine, fire truck",
  "fire screen, fireguard",
  "flagpole, flagstaff",
  "flute, transverse flute",
  "folding chair",
  "football helmet",
  "forklift",
  "fountain",
  "fountain pen",
  "four-poster",
  "freight car",
  "French horn, horn",
  "frying pan, frypan, skillet",
  "fur coat",
  "garbage truck, dustcart",
  "gasmask, respirator, gas helmet",
  "gas pump, gasoline pump, petrol pump, island dispenser",
  "goblet",
  "go-kart",
  "golf ball",
  "golfcart, golf cart",
  "gondola",
  "gong, tam-tam",
  "gown",
  "grand piano, grand",
  "greenhouse, nursery, glasshouse",
  "grille, radiator grille",
  "grocery store, grocery, food market, market",
  "guillotine",
  "hair slide",
  "hair spray",
  "half track",
  "hammer",
  "hamper",
  "hand blower, blow dryer, blow drier, hair dryer, hair drier",
  "hand-held computer, hand-held microcomputer",
  "handkerchief, hankie, hanky, hankey",
  "hard disc, hard disk, fixed disk",
  "harmonica, mouth organ, harp, mouth harp",
  "harp",
  "harvester, reaper",
  "hatchet",
  "holster",
  "home theater, home theatre",
  "honeycomb",
  "hook, claw",
  "hoopskirt, crinoline",
  "horizontal bar, high bar",
  "horse cart, horse-cart",
  "hourglass",
  "iPod",
  "iron, smoothing iron",
  "jack-o-lantern",
  "jean, blue jean, denim",
  "jeep, landrover",
  "jersey, T-shirt, tee shirt",
  "jigsaw puzzle",
  "jinrikisha, ricksha, rickshaw",
  "joystick",
  "kimono",
  "knee pad",
  "knot",
  "lab coat, laboratory coat",
  "ladle",
  "lampshade, lamp shade",
  "laptop, laptop computer",
  "lawn mower, mower",
  "lens cap, lens cover",
  "letter opener, paper knife, paperknife",
  "library",
  "lifeboat",
  "lighter, light, igniter, ignitor",
  "limousine, limo",
  "liner, ocean liner",
  "lipstick, lip rouge",
  "Loafer",
  "lotion",
  "loudspeaker, speaker, speaker unit, loudspeaker system, speaker system",
  "loupe, jeweler's loupe",
  "lumbermill, sawmill",
  "magnetic compass",
  "mailbag, postbag",
  "mailbox, letter box",
  "maillot",
  "maillot, tank suit",
  "manhole cover",
  "maraca",
  "marimba, xylophone",
  "mask",
  "matchstick",
  "maypole",
  "maze, labyrinth",
  "measuring cup",
  "medicine chest, medicine cabinet",
  "megalith, megalithic structure",
  "microphone, mike",
  "microwave, microwave oven",
  "military uniform",
  "milk can",
  "minibus",
  "miniskirt, mini",
  "minivan",
  "missile",
  "mitten",
  "mixing bowl",
  "mobile home, manufactured home",
  "Model T",
  "modem",
  "monastery",
  "monitor",
  "moped",
  "mortar",
  "mortarboard",
  "mosque",
  "mosquito net",
  "motor scooter, scooter",
  "mountain bike, all-terrain bike, off-roader",
  "mountain tent",
  "mouse, computer mouse",
  "mousetrap",
  "moving van",
  "muzzle",
  "nail",
  "neck brace",
  "necklace",
  "nipple",
  "notebook, notebook computer",
  "obelisk",
  "oboe, hautboy, hautbois",
  "ocarina, sweet potato",
  "odometer, hodometer, mileometer, milometer",
  "oil filter",
  "organ, pipe organ",
  "oscilloscope, scope, cathode-ray oscilloscope, CRO",
  "overskirt",
  "oxcart",
  "oxygen mask",
  "packet",
  "paddle, boat paddle",
  "paddlewheel, paddle wheel",
  "padlock",
  "paintbrush",
  "pajama, pyjama, pj's, jammies",
  "palace",
  "panpipe, pandean pipe, syrinx",
  "paper towel",
  "parachute, chute",
  "parallel bars, bars",
  "park bench",
  "parking meter",
  "passenger car, coach, carriage",
  "patio, terrace",
  "pay-phone, pay-station",
  "pedestal, plinth, footstall",
  "pencil box, pencil case",
  "pencil sharpener",
  "perfume, essence",
  "Petri dish",
  "photocopier",
  "pick, plectrum, plectron",
  "pickelhaube",
  "picket fence, paling",
  "pickup, pickup truck",
  "pier",
  "piggy bank, penny bank",
  "pill bottle",
  "pillow",
  "ping-pong ball",
  "pinwheel",
  "pirate, pirate ship",
  "pitcher, ewer",
  "plane, carpenter's plane, woodworking plane",
  "planetarium",
  "plastic bag",
  "plate rack",
  "plow, plough",
  "plunger, plumber's helper",
  "Polaroid camera, Polaroid Land camera",
  "pole",
  "police van, police wagon, paddy wagon, patrol wagon, wagon, black Maria",
  "poncho",
  "pool table, billiard table, snooker table",
  "pop bottle, soda bottle",
  "pot, flowerpot",
  "potter's wheel",
  "power drill",
  "prayer rug, prayer mat",
  "printer",
  "prison, prison house",
  "projectile, missile",
  "projector",
  "puck, hockey puck",
  "punching bag, punch bag, punching ball, punchball",
  "purse",
  "quill, quill pen",
  "quilt, comforter, comfort, puff",
  "racer, race car, racing car",
  "racket, racquet",
  "radiator",
  "radio, wireless",
  "radio telescope, radio reflector",
  "rain barrel",
  "recreational vehicle, RV, R.V.",
  "reel",
  "reflex camera",
  "refrigerator, icebox",
  "remote control, remote",
  "restaurant, eating house, eating place, eatery",
  "revolver, six-gun, six-shooter",
  "rifle",
  "rocking chair, rocker",
  "rotisserie",
  "rubber eraser, rubber, pencil eraser",
  "rugby ball",
  "rule, ruler",
  "running shoe",
  "safe",
  "safety pin",
  "saltshaker, salt shaker",
  "sandal",
  "sarong",
  "sax, saxophone",
  "scabbard",
  "scale, weighing machine",
  "school bus",
  "schooner",
  "scoreboard",
  "screen, CRT screen",
  "screw",
  "screwdriver",
  "seat belt, seatbelt",
  "sewing machine",
  "shield, buckler",
  "shoe shop, shoe-shop, shoe store",
  "shoji",
  "shopping basket",
  "shopping cart",
  "shovel",
  "shower cap",
  "shower curtain",
  "ski",
  "ski mask",
  "sleeping bag",
  "slide rule, slipstick",
  "sliding door",
  "slot, one-armed bandit",
  "snorkel",
  "snowmobile",
  "snowplow, snowplough",
  "soap dispenser",
  "soccer ball",
  "sock",
  "solar dish, solar collector, solar furnace",
  "sombrero",
  "soup bowl",
  "space bar",
  "space heater",
  "space shuttle",
  "spatula",
  "speedboat",
  "spider web, spider's web",
  "spindle",
  "sports car, sport car",
  "spotlight, spot",
  "stage",
  "steam locomotive",
  "steel arch bridge",
  "steel drum",
  "stethoscope",
  "stole",
  "stone wall",
  "stopwatch, stop watch",
  "stove",
  "strainer",
  "streetcar, tram, tramcar, trolley, trolley car",
  "stretcher",
  "studio couch, day bed",
  "stupa, tope",
  "submarine, pigboat, sub, U-boat",
  "suit, suit of clothes",
  "sundial",
  "sunglass",
  "sunglasses, dark glasses, shades",
  "sunscreen, sunblock, sun blocker",
  "suspension bridge",
  "swab, swob, mop",
  "sweatshirt",
  "swimming trunks, bathing trunks",
  "swing",
  "switch, electric switch, electrical switch",
  "syringe",
  "table lamp",
  "tank, army tank, armored combat vehicle, armoured combat vehicle",
  "tape player",
  "teapot",
  "teddy, teddy bear",
  "television, television system",
  "tennis ball",
  "thatch, thatched roof",
  "theater curtain, theatre curtain",
  "thimble",
  "thresher, thrasher, threshing machine",
  "throne",
  "tile roof",
  "toaster",
  "tobacco shop, tobacconist shop, tobacconist",
  "toilet seat",
  "torch",
  "totem pole",
  "tow truck, tow car, wrecker",
  "toyshop",
  "tractor",
  "trailer truck, tractor trailer, trucking rig, rig, articulated lorry, semi",
  "tray",
  "trench coat",
  "tricycle, trike, velocipede",
  "trimaran",
  "tripod",
  "triumphal arch",
  "trolleybus, trolley coach, trackless trolley",
  "trombone",
  "tub, vat",
  "turnstile",
  "typewriter keyboard",
  "umbrella",
  "unicycle, monocycle",
  "upright, upright piano",
  "vacuum, vacuum cleaner",
  "vase",
  "vault",
  "velvet",
  "vending machine",
  "vestment",
  "viaduct",
  "violin, fiddle",
  "volleyball",
  "waffle iron",
  "wall clock",
  "wallet, billfold, notecase, pocketbook",
  "wardrobe, closet, press",
  "warplane, military plane",
  "washbasin, handbasin, washbowl, lavabo, wash-hand basin",
  "washer, automatic washer, washing machine",
  "water bottle",
  "water jug",
  "water tower",
  "whiskey jug",
  "whistle",
  "wig",
  "window screen",
  "window shade",
  "Windsor tie",
  "wine bottle",
  "wing",
  "wok",
  "wooden spoon",
  "wool, woolen, woollen",
  "worm fence, snake fence, snake-rail fence, Virginia fence",
  "wreck",
  "yawl",
  "yurt",
  "web site, website, internet site, site",
  "comic book",
  "crossword puzzle, crossword",
  "street sign",
  "traffic light, traffic signal, stoplight",
  "book jacket, dust cover, dust jacket, dust wrapper",
  "menu",
  "plate",
  "guacamole",
  "consomme",
  "hot pot, hotpot",
  "trifle",
  "ice cream, icecream",
  "ice lolly, lolly, lollipop, popsicle",
  "French loaf",
  "bagel, beigel",
  "pretzel",
  "cheeseburger",
  "hotdog, hot dog, red hot",
  "mashed potato",
  "head cabbage",
  "broccoli",
  "cauliflower",
  "zucchini, courgette",
  "spaghetti squash",
  "acorn squash",
  "butternut squash",
  "cucumber, cuke",
  "artichoke, globe artichoke",
  "bell pepper",
  "cardoon",
  "mushroom",
  "Granny Smith",
  "strawberry",
  "orange",
  "lemon",
  "fig",
  "pineapple, ananas",
  "banana",
  "jackfruit, jak, jack",
  "custard apple",
  "pomegranate",
  "hay",
  "carbonara",
  "chocolate sauce, chocolate syrup",
  "dough",
  "meat loaf, meatloaf",
  "pizza, pizza pie",
  "potpie",
  "burrito",
  "red wine",
  "espresso",
  "cup",
  "eggnog",
  "alp",
  "bubble",
  "cliff, drop, drop-off",
  "coral reef",
  "geyser",
  "lakeside, lakeshore",
  "promontory, headland, head, foreland",
  "sandbar, sand bar",
  "seashore, coast, seacoast, sea-coast",
  "valley, vale",
  "volcano",
  "ballplayer, baseball player",
  "groom, bridegroom",
  "scuba diver",
  "rapeseed",
  "daisy",
  "yellow lady's slipper, yellow lady-slipper, Cypripedium calceolus, Cypripedium parviflorum",
  "corn",
  "acorn",
  "hip, rose hip, rosehip",
  "buckeye, horse chestnut, conker",
  "coral fungus",
  "agaric",
  "gyromitra",
  "stinkhorn, carrion fungus",
  "earthstar",
  "hen-of-the-woods, hen of the woods, Polyporus frondosus, Grifola frondosa",
  "bolete",
  "ear, spike, capitulum",
  "toilet tissue, toilet paper, bathroom tissue"
};

typedef struct {

    /* OpenVX references */
    vx_context context;

    vx_graph   graph;

    vx_user_data_object         config;

    sTIDL_IOBufDesc_t           ioBufDesc;

    TIOVXDLPreProcModuleObj     dlPreProcObj;

    TIOVXTIDLModuleObj          tidlObj;

    TIOVXDLPostProcModuleObj    dlPostProcObj;

} AppObj;

static AppObj gAppObj;

static vx_status app_init(AppObj *obj);
static void app_deinit(AppObj *obj);
static vx_status app_create_graph(AppObj *obj);
static vx_status app_verify_graph(AppObj *obj);
static vx_status app_run_graph(AppObj *obj);
static void app_delete_graph(AppObj *obj);

static vx_user_data_object readConfig(AppObj *obj, vx_context context, char *config_file);

vx_status app_modules_dl_pre_proc_tidl_dl_post_proc_test(vx_int32 argc, vx_char* argv[])
{
    AppObj *obj = &gAppObj;
    vx_status status = VX_SUCCESS;

    status = app_init(obj);
    APP_PRINTF("App Init Done! \n");

    if(status == VX_SUCCESS)
    {
        status = app_create_graph(obj);
        APP_PRINTF("App Create Graph Done! \n");
    }

    if(status == VX_SUCCESS)
    {
        status = app_verify_graph(obj);
        APP_PRINTF("App Verify Graph Done! \n");
    }
    if (status == VX_SUCCESS)
    {
        status = app_run_graph(obj);
        APP_PRINTF("App Run Graph Done! \n");
    }

    app_delete_graph(obj);
    APP_PRINTF("App Delete Graph Done! \n");

    app_deinit(obj);
    APP_PRINTF("App De-init Done! \n");

    return status;
}

static vx_status app_init(AppObj *obj)
{
    vx_status status = VX_SUCCESS;

    /* Create OpenVx Context */
    obj->context = vxCreateContext();
    status = vxGetStatus((vx_reference) obj->context);

    if(status == VX_SUCCESS)
    {
        tivxEdgeaiImgProcLoadKernels(obj->context);
        tivxTIDLLoadKernels(obj->context);
    }

    /* Pre Proc Init */
    if(status == VX_SUCCESS)
    {
        obj->config = readConfig(obj, obj->context, TIDL_CONFIG_FILE_PATH);

        if (obj->ioBufDesc.inWidth[0] != IMAGE_WIDTH)
        {
            APP_ERROR("Input tensor width [%d] does not match image width [%d]", obj->ioBufDesc.inWidth[0], IMAGE_WIDTH);
            return VX_ERROR_INVALID_PARAMETERS;
        }

        if (obj->ioBufDesc.inHeight[0] != IMAGE_HEIGHT)
        {
            APP_ERROR("Input tensor height [%d] does not match image height [%d]", obj->ioBufDesc.inHeight[0], IMAGE_HEIGHT);
            return VX_ERROR_INVALID_PARAMETERS;
        }

        TIOVXDLPreProcModuleObj *dlPreProcObj = &obj->dlPreProcObj;

        /* These specifications are particular to this model. */
        dlPreProcObj->params.channel_order = TIVX_DL_PRE_PROC_CHANNEL_ORDER_NCHW;
        dlPreProcObj->params.tensor_format = TIVX_DL_PRE_PROC_TENSOR_FORMAT_BGR;
   
        dlPreProcObj->params.scale[0] = obj->ioBufDesc.inTensorScale[0]; //For R or Y plane
        dlPreProcObj->params.scale[1] = obj->ioBufDesc.inTensorScale[0]; //For G or U plane
        dlPreProcObj->params.scale[2] = obj->ioBufDesc.inTensorScale[0]; //For B or V plane

        dlPreProcObj->params.mean[0] = 0.0; //For R or Y plane
        dlPreProcObj->params.mean[1] = 0.0; //For G or U plane
        dlPreProcObj->params.mean[2] = 0.0; //For B or V plane

        dlPreProcObj->params.crop[0] = 0; //Top
        dlPreProcObj->params.crop[1] = 0; //Bottom
        dlPreProcObj->params.crop[2] = 0; //Left
        dlPreProcObj->params.crop[3] = 0; //Right

        dlPreProcObj->num_channels = 1;
        dlPreProcObj->input.bufq_depth = APP_BUFQ_DEPTH;
        dlPreProcObj->input.color_format = VX_DF_IMAGE_NV12;
        dlPreProcObj->input.width = IMAGE_WIDTH;
        dlPreProcObj->input.height = IMAGE_HEIGHT;

        dlPreProcObj->output.bufq_depth = APP_BUFQ_DEPTH;
        dlPreProcObj->output.datatype = VX_TYPE_UINT8;
        dlPreProcObj->output.num_dims = 3;

        dlPreProcObj->output.dim_sizes[0] = IMAGE_WIDTH;
        dlPreProcObj->output.dim_sizes[1] = IMAGE_HEIGHT;
        dlPreProcObj->output.dim_sizes[2] = 3;

        dlPreProcObj->en_out_tensor_write = 0;

        /* Initialize modules */
        status = tiovx_dl_pre_proc_module_init(obj->context, dlPreProcObj);
        APP_PRINTF("DLPreProc Init Done! \n");
    }

    /* TIDL Init */
    if(status == VX_SUCCESS)
    {
        TIOVXTIDLModuleObj *tidlObj = &obj->tidlObj;

        tidlObj->config_file_path = TIDL_CONFIG_FILE_PATH;
        tidlObj->config = obj->config;

        tidlObj->num_input_tensors  = obj->ioBufDesc.numInputBuf;
        tidlObj->num_output_tensors = obj->ioBufDesc.numOutputBuf;

        tidlObj->input[0].bufq_depth = APP_BUFQ_DEPTH;
        for(uint32_t i=0; i < tidlObj->num_output_tensors; i++)
        {
            tidlObj->output[i].bufq_depth = APP_BUFQ_DEPTH;
        }
        tidlObj->network_file_path = TIDL_NETWORK_FILE_PATH;
        tidlObj->num_cameras = 1;
        tidlObj->en_out_tensor_write = 0;

        /* Initialize modules */
        status = tiovx_tidl_module_init(obj->context, tidlObj, "tidlObj");
        APP_PRINTF("TIDL Init Done! \n");
    }

    /* Post Proc Init */
    if(status == VX_SUCCESS)
    {
        TIOVXDLPostProcModuleObj *dlPostProcObj = &obj->dlPostProcObj;

        dlPostProcObj->params.task_type = TIVX_DL_POST_PROC_CLASSIFICATION_TASK_TYPE;
        
        dlPostProcObj->params.oc_prms.ioBufDesc = &obj->ioBufDesc;
        dlPostProcObj->params.oc_prms.labelOffset = 0;
        dlPostProcObj->params.oc_prms.num_top_results = 5;

        // Classnames
        for(uint32_t i=0; i < 1001; i++)
        {
            strcpy(dlPostProcObj->params.oc_prms.classnames[i], imgnet_labels[i]);
        }

        dlPostProcObj->params.num_input_tensors = obj->ioBufDesc.numOutputBuf;

        dlPostProcObj->num_input_tensors = obj->ioBufDesc.numOutputBuf;
        dlPostProcObj->num_channels = 1;
        dlPostProcObj->input_image.bufq_depth = APP_BUFQ_DEPTH;
        dlPostProcObj->input_image.color_format = VX_DF_IMAGE_NV12;

        dlPostProcObj->input_image.width = IMAGE_WIDTH;
        dlPostProcObj->input_image.height = IMAGE_HEIGHT;

        for(uint32_t i=0; i < dlPostProcObj->num_input_tensors; i++)
        {
            dlPostProcObj->input_tensor[i].bufq_depth = APP_BUFQ_DEPTH;
            dlPostProcObj->input_tensor[i].datatype = VX_TYPE_FLOAT32;
            dlPostProcObj->input_tensor[i].num_dims = 3;
            
            dlPostProcObj->input_tensor[i].dim_sizes[0] = obj->ioBufDesc.outWidth[i]  +
                                                          obj->ioBufDesc.outPadL[i]   +
                                                          obj->ioBufDesc.outPadR[i];

            dlPostProcObj->input_tensor[i].dim_sizes[1] = obj->ioBufDesc.outHeight[i] +
                                                          obj->ioBufDesc.outPadT[i]   +
                                                          obj->ioBufDesc.outPadB[i];

            dlPostProcObj->input_tensor[i].dim_sizes[2] = obj->ioBufDesc.outNumChannels[i];
        }

        dlPostProcObj->en_out_image_write = 0;
        dlPostProcObj->output_image.bufq_depth = APP_BUFQ_DEPTH;
        dlPostProcObj->output_image.color_format = VX_DF_IMAGE_NV12;

        dlPostProcObj->output_image.width = IMAGE_WIDTH;
        dlPostProcObj->output_image.height = IMAGE_HEIGHT;

        /* Initialize modules */
        status = tiovx_dl_post_proc_module_init(obj->context, dlPostProcObj);
        APP_PRINTF("DLPostProc Init Done! \n");
    }

    return status;
}

static void app_deinit(AppObj *obj)
{
    tiovx_dl_pre_proc_module_deinit(&obj->dlPreProcObj);
    tiovx_tidl_module_deinit(&obj->tidlObj);
    tiovx_dl_post_proc_module_deinit(&obj->dlPostProcObj);

    tivxEdgeaiImgProcUnLoadKernels(obj->context);
    tivxTIDLUnLoadKernels(obj->context);

    vxReleaseContext(&obj->context);
}

static void app_delete_graph(AppObj *obj)
{
    tiovx_dl_pre_proc_module_delete(&obj->dlPreProcObj);
    tiovx_tidl_module_delete(&obj->tidlObj);
    tiovx_dl_post_proc_module_delete(&obj->dlPostProcObj);

    vxReleaseGraph(&obj->graph);
}

static vx_status app_create_graph(AppObj *obj)
{
    vx_status status = VX_SUCCESS;

    vx_graph_parameter_queue_params_t graph_parameters_queue_params_list[8];
    vx_int32 graph_parameter_index;

    obj->graph = vxCreateGraph(obj->context);
    status = vxGetStatus((vx_reference)obj->graph);

    if((vx_status)VX_SUCCESS == status)
    {
        status = tiovx_dl_pre_proc_module_create(obj->graph, &obj->dlPreProcObj, NULL, TIVX_TARGET_MPU_0);
    }

    if((vx_status)VX_SUCCESS == status)
    {
        status = tiovx_tidl_module_create(obj->context, obj->graph, &obj->tidlObj, obj->dlPreProcObj.output.arr);
    }

    if((vx_status)VX_SUCCESS == status)
    {
        status = tiovx_dl_post_proc_module_create(obj->graph, &obj->dlPostProcObj, obj->dlPreProcObj.input.arr[0], obj->tidlObj.output, TIVX_TARGET_MPU_0);
    }

    graph_parameter_index = 0;
    if((vx_status)VX_SUCCESS == status)
    {
        status = add_graph_parameter_by_node_index(obj->graph, obj->dlPreProcObj.node, 1);
        obj->dlPreProcObj.input.graph_parameter_index = graph_parameter_index;
        graph_parameters_queue_params_list[graph_parameter_index].graph_parameter_index = graph_parameter_index;
        graph_parameters_queue_params_list[graph_parameter_index].refs_list_size = APP_BUFQ_DEPTH;
        graph_parameters_queue_params_list[graph_parameter_index].refs_list = (vx_reference*)&obj->dlPreProcObj.input.image_handle[0];
        graph_parameter_index++;
    }

    if((vx_status)VX_SUCCESS == status)
    {
        status = add_graph_parameter_by_node_index(obj->graph, obj->dlPostProcObj.node, 2);
        obj->dlPostProcObj.output_image.graph_parameter_index = graph_parameter_index;
        graph_parameters_queue_params_list[graph_parameter_index].graph_parameter_index = graph_parameter_index;
        graph_parameters_queue_params_list[graph_parameter_index].refs_list_size = APP_BUFQ_DEPTH;
        graph_parameters_queue_params_list[graph_parameter_index].refs_list = (vx_reference*)&obj->dlPostProcObj.output_image.image_handle[0];
        graph_parameter_index++;
    }

    if((vx_status)VX_SUCCESS == status)
    {
        status = vxSetGraphScheduleConfig(obj->graph,
                    VX_GRAPH_SCHEDULE_MODE_QUEUE_MANUAL,
                    graph_parameter_index,
                    graph_parameters_queue_params_list);
    }

    return status;
}

static vx_status app_verify_graph(AppObj *obj)
{
    vx_status status = VX_SUCCESS;

    status = vxVerifyGraph(obj->graph);

    APP_PRINTF("App Verify Graph Done!\n");

    return status;
}

static vx_status app_run_graph(AppObj *obj)
{
    vx_status status = VX_SUCCESS;

    char input_filename[100];
    char output_filename[100];

    sprintf(input_filename, "%s/raw_images/modules_test/baboon_224x224_nv12.yuv", EDGEAI_DATA_PATH);
    sprintf(output_filename, "%s/output/dl-post-proc-output.yuv", EDGEAI_DATA_PATH);

    vx_image input_o, output_o;
    uint32_t num_refs;

    TIOVXDLPreProcModuleObj     *dlPreProcObj = &obj->dlPreProcObj;
    TIOVXDLPostProcModuleObj    *dlPostProcObj = &obj->dlPostProcObj;

    readImage(input_filename, dlPreProcObj->input.image_handle[0]);

    APP_PRINTF("Enqueueing input buffers!\n");
    vxGraphParameterEnqueueReadyRef(obj->graph, dlPreProcObj->input.graph_parameter_index, (vx_reference*)&dlPreProcObj->input.image_handle[0], 1);
    APP_PRINTF("Enqueueing output buffers!\n");
    vxGraphParameterEnqueueReadyRef(obj->graph, dlPostProcObj->output_image.graph_parameter_index, (vx_reference*)&dlPostProcObj->output_image.image_handle[0], 1);

    APP_PRINTF("Processing!\n");
    status = vxScheduleGraph(obj->graph);
    if((vx_status)VX_SUCCESS != status) {
      APP_ERROR("Schedule Graph failed: %d!\n", status);
    }
    status = vxWaitGraph(obj->graph);
    if((vx_status)VX_SUCCESS != status) {
      APP_ERROR("Wait Graph failed: %d!\n", status);
    }
    
    APP_PRINTF("Dequeueing input buffers!\n");
    vxGraphParameterDequeueDoneRef(obj->graph, dlPreProcObj->input.graph_parameter_index, (vx_reference*)&input_o, 1, &num_refs);
    APP_PRINTF("Dequeueing output buffers!\n");
    vxGraphParameterDequeueDoneRef(obj->graph, dlPostProcObj->output_image.graph_parameter_index, (vx_reference*)&output_o, 1, &num_refs);

    writeImage(output_filename, dlPostProcObj->output_image.image_handle[0]);

    return status;
}

static vx_user_data_object readConfig(AppObj *obj, vx_context context, char *config_file)
{
    vx_status status = VX_SUCCESS;

    tivxTIDLJ7Params  *tidlParams = NULL;
    sTIDL_IOBufDesc_t *ioBufDesc = NULL;
    vx_user_data_object   config = NULL;
    vx_uint32 capacity;
    vx_map_id map_id;

    FILE *fp_config;
    vx_size read_count;

    APP_PRINTF("Reading config file %s ...\n", config_file);

    fp_config = fopen(config_file, "rb");

    if(fp_config == NULL)
    {
        APP_ERROR("ERROR: Unable to open IO config file %s \n", config_file);

        return NULL;
    }

    fseek(fp_config, 0, SEEK_END);
    capacity = ftell(fp_config);
    fseek(fp_config, 0, SEEK_SET);

    if( capacity != sizeof(sTIDL_IOBufDesc_t) )
    {
        APP_ERROR("Config file size (%d bytes) does not match size of sTIDL_IOBufDesc_t (%d bytes)\n", capacity, (vx_uint32)sizeof(sTIDL_IOBufDesc_t));
        fclose(fp_config);
        return NULL;
    }


    /* Create a user struct type for handling config data*/
    config = vxCreateUserDataObject(context, "tivxTIDLJ7Params", sizeof(tivxTIDLJ7Params), NULL );

    status = vxGetStatus((vx_reference)config);

    if (VX_SUCCESS == status)
    {
        status = vxMapUserDataObject(config, 0, sizeof(tivxTIDLJ7Params), &map_id,
                            (void **)&tidlParams, VX_WRITE_ONLY, VX_MEMORY_TYPE_HOST, 0);

        if (VX_SUCCESS == status)
        {
            if(tidlParams == NULL)
            {
              APP_ERROR("Map of config object failed\n");
              fclose(fp_config);
              return NULL;
            }

            tivx_tidl_j7_params_init(tidlParams);

            ioBufDesc = (sTIDL_IOBufDesc_t *)&tidlParams->ioBufDesc;

            read_count = fread(ioBufDesc, capacity, 1, fp_config);
            if(read_count != 1)
            {
              APP_ERROR("Unable to read config file\n");
            }
            fclose(fp_config);

            memcpy(&obj->ioBufDesc, ioBufDesc, capacity);

            if(status == VX_SUCCESS)
            {
                status = vxUnmapUserDataObject(config, map_id);
            }
        }
        else
        {
            fclose(fp_config);
        }
    }
    else
    {
        fclose(fp_config);
    }

    APP_PRINTF("Reading config file %s ... Done. %d bytes\n", config_file, (uint32_t)capacity);
    APP_PRINTF("Tensors, input = %d, output = %d\n", ioBufDesc->numInputBuf,  ioBufDesc->numOutputBuf);

    return config;
}