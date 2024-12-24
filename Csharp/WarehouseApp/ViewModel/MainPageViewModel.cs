using System.Collections.ObjectModel;
using WarehouseApp.ViewModel;

namespace WarehouseApp.ViewModel;

public class MainPageViewModel
{
    public ObservableCollection<CardViewModel> Items { get; }

    public MainPageViewModel()
    {
        Items = new ObservableCollection<CardViewModel>
        {
            new CardViewModel("Прокладки", "prokladki.png"),
            new CardViewModel("Болты", "bolt.png"),
            new CardViewModel("Гайки", "gayki.png"),
            new CardViewModel("Бруски", "brusok.png"),
            new CardViewModel("Шайбы", "shayba.png"),
        };
    }
}
